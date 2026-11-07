// One outstanding MMIO transaction. Payloads remain held through the
// synchronized request/acknowledge exchange; no raw CNN data reaches CPU.
`timescale 1ns/1ps
module cnn_cdc_bridge /* synthesis syn_hier="hard" */ (
    input wire c_clk,c_resetn,
    input wire c_start,c_write,
    input wire [11:0] c_addr,
    input wire [31:0] c_wdata,
    output reg c_busy,c_done,
    (* syn_preserve=1 *) output reg [31:0] c_rdata,
    input wire m_clk,m_resetn,
    output wire m_sel,m_we,
    output wire [11:0] m_addr,
    output wire [31:0] m_wdata,
    input wire [31:0] m_rdata
);
    (* syn_preserve=1 *) reg [44:0] c_req_hold;
    (* syn_preserve=1 *) reg [44:0] m_req_hold;
    (* syn_preserve=1 *) reg [31:0] m_rsp_hold;
    reg c_req_toggle,m_ack_toggle,m_pending;
    (* ASYNC_REG="TRUE",syn_preserve=1 *) reg m_req_sync1,m_req_sync2;
    (* ASYNC_REG="TRUE",syn_preserve=1 *) reg c_ack_sync1,c_ack_sync2;

    assign m_addr=m_req_hold[43:32];
    assign m_wdata=m_req_hold[31:0];
    assign m_sel=m_pending;
    assign m_we=m_pending & m_req_hold[44];

    always @(posedge c_clk or negedge c_resetn) begin
        if(!c_resetn) begin
            c_ack_sync1<=0;c_ack_sync2<=0;
            c_req_toggle<=0;c_req_hold<=0;
            c_busy<=0;c_done<=0;c_rdata<=0;
        end else begin
            c_ack_sync1<=m_ack_toggle;c_ack_sync2<=c_ack_sync1;
            c_done<=0;
            if(c_start && !c_busy) begin
                c_req_hold<={c_write,c_addr,c_wdata};
                c_req_toggle<=~c_req_toggle;
                c_busy<=1;
            end else if(c_busy && c_ack_sync2==c_req_toggle) begin
                // m_rsp_hold was stable before the acknowledge traversed
                // both synchronizer stages. Capture it once, then signal.
                c_rdata<=m_rsp_hold;c_done<=1;c_busy<=0;
            end
`ifdef FACEREC_SIM
            if(c_start && c_busy) $fatal(1,"CDC request overlaps outstanding transaction");
`endif
        end
    end

    always @(posedge m_clk or negedge m_resetn) begin
        if(!m_resetn) begin
            m_req_sync1<=0;m_req_sync2<=0;
            m_req_hold<=0;m_rsp_hold<=0;
            m_ack_toggle<=0;m_pending<=0;
        end else begin
            m_req_sync1<=c_req_toggle;m_req_sync2<=m_req_sync1;
            if(m_pending) begin
                // The MMIO write commits at this edge. Read data has had
                // a complete destination cycle to decode the latched address.
                m_rsp_hold<=m_rdata;
                m_ack_toggle<=m_req_sync2;
                m_pending<=0;
            end else if(m_req_sync2!=m_ack_toggle) begin
                m_req_hold<=c_req_hold;
                m_pending<=1;
            end
        end
    end
endmodule
