#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""pack_frames.py —— 由图片生成 cnn_framesrc 的帧 ROM（framesrc.mem）

格式（与 rtl/accel/cnn_framesrc.v 一致）：
  · 每行一个 24b 字：``%06X`` = {R[7:0], G[7:0], B[7:0]}，R 在高字节
  · 像素按 ``y*FW + x`` 顺序排列，每帧 NPIX = FW*FH 个有效像素
  · 每帧占 STRIDE 个字的对齐跨度（默认 16384），不足补 0
  · NFRAME 帧首尾相接

用法：
  python tools/pack_frames.py --out rtl/accel/mem/framesrc.mem a.png b.png c.png d.png
  python tools/pack_frames.py --synthetic --out rtl/accel/mem/framesrc.mem
  python tools/pack_frames.py --synthetic --frames 8 --out ...

从图片读入需要 Pillow：``pip install pillow``。
"""
import argparse
import os
import sys


def parse_args():
    ap = argparse.ArgumentParser(description='生成 cnn_framesrc 帧 ROM')
    ap.add_argument('images', nargs='*', help='输入图片（任意尺寸，自动缩放）')
    ap.add_argument('--out', required=True, help='输出 .mem 路径')
    ap.add_argument('--fw', type=int, default=128, help='帧宽（默认 128）')
    ap.add_argument('--fh', type=int, default=96, help='帧高（默认 96）')
    ap.add_argument('--stride', type=int, default=16384, help='每帧对齐跨度（默认 16384）')
    ap.add_argument('--frames', type=int, default=0, help='帧数（synthetic 时默认 4）')
    ap.add_argument('--synthetic', action='store_true', help='不读图片，生成测试图案')
    return ap.parse_args()


def frame_from_image(path, fw, fh):
    try:
        from PIL import Image
    except ImportError:
        sys.exit('需要 Pillow：pip install pillow')
    im = Image.open(path).convert('RGB').resize((fw, fh), Image.LANCZOS)
    px = im.load()
    words = []
    for y in range(fh):
        for x in range(fw):
            r, g, b = px[x, y]
            words.append((r << 16) | (g << 8) | b)
    return words


def frame_synthetic(idx, fw, fh):
    """梯度背景 + 随帧移动的高亮方块，用于冒烟测试。"""
    words = []
    box_x = (idx * fw) // 8
    for y in range(fh):
        for x in range(fw):
            r = (x * 255) // max(1, fw - 1)
            g = (y * 255) // max(1, fh - 1)
            b = 64 + 32 * idx
            if box_x <= x < box_x + fw // 8 and fh // 3 <= y < 2 * fh // 3:
                r, g, b = 255, 255, 255
            words.append((r << 16) | (g << 8) | b)
    return words


def main():
    args = parse_args()
    if args.synthetic:
        n = args.frames or 4
        frames = [frame_synthetic(i, args.fw, args.fh) for i in range(n)]
        src_desc = 'synthetic x%d' % n
    else:
        if not args.images:
            sys.exit('请提供图片，或使用 --synthetic')
        frames = [frame_from_image(p, args.fw, args.fh) for p in args.images]
        src_desc = ', '.join(os.path.basename(p) for p in args.images)

    out = os.path.abspath(args.out)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    npix = args.fw * args.fh
    assert npix <= args.stride, 'STRIDE 必须 >= FW*FH'
    lines = []
    for words in frames:
        for w in words:
            lines.append('%06X' % (w & 0xFFFFFF))
        for _ in range(args.stride - len(words)):
            lines.append('000000')
    with open(out, 'w', encoding='ascii', newline='') as f:
        f.write('\n'.join(lines))
        f.write('\n')
    total = len(frames) * args.stride
    print('[ok] %s: %d frames, %d px/frame, stride %d, %d words (%d bytes) <- %s'
          % (out, len(frames), npix, args.stride, total, os.path.getsize(out), src_desc))


if __name__ == '__main__':
    main()
