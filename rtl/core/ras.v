//=====================================================================
// ras.v : 返回地址栈 (Return Address Stack, S1 预测器强化 2026-10-11)
//
// 16 项 LIFO。call (jal/jalr 且 rd=x1/x5) 在 EX 解析拍压入返回地址
// (pc+4); ret (jalr 且 rs1=x1/x5, rd=x0) 在 EX 解析拍弹出。
// IF 预测拍只 peek 不消耗 (流水冻结会对同一 pc 反复预测, peek 幂等)。
//
// 空栈 peek: peek_v=0, peek 值为垃圾 (消费侧必须门控) —— 上层用
// `btype==ret && peek_v` 决定是否预测 taken, 落空走 EX 重定向兜底。
// 深度超 16: push 被抑制 (递归深度 >16 的最内层返回走 EX 兜底)。
// push/pop 互斥: 每拍至多一次 RAS 操作 —— ★ P2 松绑 (2026-10-11 §5d) 后
// slot1 也可以是 call/ret, 同拍双操作由上层仲裁 (slot0 程序序更老, 优先;
// slot1 的让位只损预测质量不出错), 本模块仍只见单 push/单 pop。
//=====================================================================
`timescale 1ns/1ps

module ras #(
    parameter FORWARD_PENDING = 0
)(
    input  wire        clk,
    input  wire        resetn,
    input  wire        push,
    input  wire [31:0] push_data,
    input  wire        pop,
    output wire        peek_v,
    output wire [31:0] peek
);
    localparam DEPTH = 16;

    reg [31:0] mem [0:DEPTH-1];
    reg [ 4:0] sp;                 // 栈内项数 0..16

    // With registered command inputs, expose the state after that pending
    // command while physical storage applies it at the next edge. This
    // restores the legacy cycle-visible top without a late EX-to-sp CE path.
    // Match the original priority, including push-full plus pop => pop.
    wire pending_push = (push === 1'b1) && (sp < DEPTH);
    wire pending_pop = !pending_push && (pop === 1'b1) && (sp != 5'd0);
    wire [4:0] visible_sp = pending_push ? (sp + 5'd1)
                            : pending_pop ? (sp - 5'd1) : sp;
    wire [31:0] physical_top = mem[sp[3:0] - 4'd1];
    wire [31:0] popped_top = mem[sp[3:0] - 4'd2];
    assign peek_v = FORWARD_PENDING ? (visible_sp != 5'd0) : (sp != 5'd0);
    assign peek = FORWARD_PENDING
        ? (pending_push ? push_data : (pending_pop ? popped_top : physical_top))
        : physical_top;

    always @(posedge clk) begin
        if (!resetn)
            sp <= 5'd0;
        else if (push && (sp < DEPTH)) begin
            mem[sp[3:0]] <= push_data;
            sp           <= sp + 5'd1;
        end else if (pop && (sp != 5'd0))
            sp <= sp - 5'd1;
    end
endmodule
