# -*- coding: utf-8 -*-
"""扫描 GBK 源文件里 '注释/字符串之外出现高位字节' 这一类破坏（VER=82 插入注释块时
把原注释块的续行挤到注释外，GCC 报 stray '\\272'）。"""
import sys, io, os

ENCS = ['gbk', 'utf-8']


def decode(data):
    for e in ENCS:
        try:
            return data.decode(e), e
        except UnicodeDecodeError:
            continue
    return None, None


def scan(path, show=6):
    data = open(path, 'rb').read()
    text, enc = decode(data)
    if text is None:
        print('  [!!] %s 两种编码都解不开' % os.path.basename(path))
        return 1
    i = 0
    n = len(text)
    state = 'code'
    line = 1
    start_line = 1
    bad = []
    lines = text.split('\n')
    while i < n:
        c = text[i]
        if state == 'code':
            if text.startswith('/*', i):
                state, start_line, i = 'comment', line, i + 2
                continue
            if text.startswith('//', i):
                state, i = 'line', i + 2
                continue
            if c == '"':
                state, i = 'str', i + 1
                continue
            if c == "'":
                state, i = 'chr', i + 1
                continue
            if ord(c) > 127:
                bad.append((line, c, lines[line - 1].strip()[:70]))
        elif state == 'comment':
            if text.startswith('*/', i):
                state, i = 'code', i + 2
                continue
        elif state == 'line':
            if c == '\n':
                state = 'code'
        elif state == 'str':
            if c == '\\':
                i += 2
                continue
            if c == '"':
                state = 'code'
        elif state == 'chr':
            if c == '\\':
                i += 2
                continue
            if c == "'":
                state = 'code'
        if c == '\n':
            line += 1
        i += 1

    name = os.path.basename(path)
    tail = '' if state == 'code' else '  [!!] 文件结束时仍在 %s 状态(起于第 %d 行)' % (state, start_line)
    print('  %-18s enc=%-6s stray=%-4d %s' % (name, enc, len(bad), tail))
    for (ln, ch, txt) in bad[:show]:
        print('        L%-5d U+%04X %s' % (ln, ord(ch), txt))
    if len(bad) > show:
        print('        ... 共 %d 处' % len(bad))
    return len(bad) + (0 if state == 'code' else 1)


if __name__ == '__main__':
    tot = 0
    for p in sys.argv[1:]:
        tot += scan(p)
    print('  TOTAL problems = %d' % tot)
