# -*- coding: utf-8 -*-
"""Minimal MATLAB v5 MAT-file reader (numeric/char/cell/struct), numpy only."""
import struct
import zlib

import numpy as np

MI_DOUBLE = 6
MI_SINGLE = 7
MI_INT8 = 1
MI_UINT8 = 2
MI_INT16 = 3
MI_UINT16 = 4
MI_INT32 = 5
MI_UINT32 = 6
MI_MATRIX = 14
MI_COMPRESSED = 15

DTYPES = {
    1: 'i1', 2: 'u1', 3: 'i2', 4: 'u2', 5: 'i4', 6: 'u4',
    7: 'f4', 9: 'f8', 12: 'i8', 13: 'u8', 16: 'U1',
}
MCLASS_DTYPE = {6: 'f8', 7: 'f4', 8: 'i1', 9: 'u1', 10: 'i2',
                11: 'u2', 12: 'i4', 13: 'u4', 14: 'i8', 15: 'u8'}


def _tag(buf, pos):
    """Return (type, nbytes, data_start, next_pos). Handles small-data-element."""
    t, n = struct.unpack_from('<II', buf, pos)
    if t >> 16:  # small data element: type in low 16 bits of first word
        t = t & 0xFFFF
        return t, n, pos + 4, pos + 8
    return t, n, pos + 8, pos + 8 + n + ((8 - n % 8) % 8)


def _read_array(buf, pos):
    """Parse one miMATRIX element starting at tag position; return (value, next_pos)."""
    t, n, p, nxt = _tag(buf, pos)
    assert t == MI_MATRIX, 'expected miMATRIX, got %d' % t
    end = p + n
    # array flags
    ft, fn, fp, _ = _tag(buf, p)
    flags = struct.unpack_from('<I', buf, fp)[0]
    mclass = flags & 0xFF
    # dimensions
    dt, dn, dp, _ = _tag(buf, fp + 8)
    dims = struct.unpack_from('<%di' % (dn // 4), buf, dp)
    dims = tuple(d for d in dims if d > 0) or (1, 1)
    # name
    nt, nn, np_, nnx = _tag(buf, dp + dn + ((8 - dn % 8) % 8))
    name = buf[np_:np_ + nn].decode('latin-1')
    p = nnx

    if mclass in (6, 7, 8, 9, 10, 11, 12, 13, 14, 15):
        ct, cn, cp, _ = _tag(buf, p)
        dt_ = MCLASS_DTYPE[mclass]
        if ct in DTYPES and ct != 16:
            dt_ = DTYPES[ct]
        arr = np.frombuffer(buf, dtype=np.dtype(dt_), count=int(np.prod(dims)), offset=cp)
        arr = arr.reshape(dims, order='F')
        return arr, nxt

    if mclass == 4:  # char
        ct, cn, cp, _ = _tag(buf, p)
        raw = buf[cp:cp + cn].decode('latin-1')
        rows = dims[0] if len(dims) == 2 else 1
        cols = dims[1] if len(dims) == 2 else dims[0]
        if rows > 1:
            elems = [raw[i * cols:(i + 1) * cols] for i in range(rows)]
        else:
            elems = [raw]
        val = elems if len(elems) > 1 else elems[0]
        return val, nxt

    if mclass == 1:  # cell
        flat = []
        cp = p
        for _ in range(int(np.prod(dims))):
            v, cp = _read_array(buf, cp)
            flat.append(v)
        return np.array(flat, dtype=object).reshape(dims, order='F'), nxt

    if mclass == 2:  # struct
        ft, fn2, fp2, fnx2 = _tag(buf, p)
        flen = struct.unpack_from('<I', buf, fp2)[0]     # max field-name length
        nt2, nn2, np2, nnx2 = _tag(buf, fnx2)            # concatenated field names
        nfields = nn2 // flen
        fields = []
        for i in range(nfields):
            raw_name = buf[np2 + i * flen:np2 + (i + 1) * flen]
            fields.append(raw_name.split(b'\x00')[0].decode('latin-1'))
        p = nnx2
        out = {}
        for i in range(int(np.prod(dims))):
            rec = {}
            for f in fields:
                v, p = _read_array(buf, p)
                rec[f] = v
            out[i if np.prod(dims) > 1 else 0] = rec
        return (out if np.prod(dims) > 1 else out[0]), nxt

    raise NotImplementedError('class %s' % MCLASS.get(mclass, mclass))


def loadmat(path):
    buf = open(path, 'rb').read()
    assert buf[:4] == b'MATL', 'not a MAT v5 file'
    pos = 128
    out = {}
    while pos < len(buf) - 8:
        t, n, p, nxt = _tag(buf, pos)
        if t == MI_COMPRESSED:
            raw = zlib.decompress(buf[p:p + n])
            v, _ = _read_array(raw, 0)
            # name is inside the decompressed element
            out[_name_of_element(raw)] = v
        elif t == MI_MATRIX:
            v, _ = _read_array(buf, pos)
            out[_name_of_element(buf[pos:])] = v
        elif t == 0:
            break
        else:
            pass
        pos = nxt
    return out


def _name_of_element(buf):
    ct, cn, cp, _ = _tag(buf, 0)  # miMATRIX tag
    ft, fn, fp, fnx = _tag(buf, cp)
    dt, dn, dp, _ = _tag(buf, fp + 8)
    nt, nn, np_, nnx = _tag(buf, dp + dn + ((8 - dn % 8) % 8))
    return buf[np_:np_ + nn].decode('latin-1')


if __name__ == '__main__':
    import sys
    d = loadmat(sys.argv[1])
    for k, v in d.items():
        if isinstance(v, np.ndarray):
            print(k, v.dtype, v.shape, 'min=%.6g max=%.6g' % (np.nanmin(v), np.nanmax(v)))
        else:
            print(k, type(v))
