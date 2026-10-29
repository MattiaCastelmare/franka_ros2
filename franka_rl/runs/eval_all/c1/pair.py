"""Paired McNemar between two trace sets: python3 c1/pair.py GLOB_A GLOB_B   ({sc} placeholder for static/dynamic)."""
import json, glob, sys
from math import comb
def load(p): return {e['seed']: e for f in glob.glob(p) for e in json.load(open(f))['eps']}
def mc(b, c):
    n = b + c; return 1.0 if n == 0 else min(1.0, 2 * sum(comb(n, i) for i in range(min(b, c) + 1)) / 2 ** n)
for sc in ('dynamic', 'static'):
    A, B = load(sys.argv[1].format(sc=sc)), load(sys.argv[2].format(sc=sc)); S = sorted(set(A) & set(B))
    for k in ('held', 'coll'):
        b = sum(A[s][k] and not B[s][k] for s in S); c = sum(B[s][k] and not A[s][k] for s in S)
        print(f'{sc:7s} {k}: A {sum(A[s][k] for s in S)} B {sum(B[s][k] for s in S)} (n={len(S)})  A-only {b} B-only {c}  p={mc(b, c):.4f}')
