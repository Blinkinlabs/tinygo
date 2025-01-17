// CGo errors:
//     testdata/errors.go:14:1: missing function name in #cgo noescape line
//     testdata/errors.go:15:1: multiple function names in #cgo noescape line
//     testdata/errors.go:4:2: warning: some warning
//     testdata/errors.go:11:9: error: unknown type name 'someType'
//     testdata/errors.go:31:5: warning: another warning
//     testdata/errors.go:18:23: unexpected token ), expected end of expression
//     testdata/errors.go:26:26: unexpected token ), expected end of expression
//     testdata/errors.go:21:33: unexpected token ), expected end of expression
//     testdata/errors.go:22:34: unexpected token ), expected end of expression
//     -: unexpected token INT, expected end of expression
//     testdata/errors.go:35:35: unexpected number of parameters: expected 2, got 3
//     testdata/errors.go:36:31: unexpected number of parameters: expected 2, got 1
//     testdata/errors.go:3:1: function "unusedFunction" in #cgo noescape line is not used

// Type checking errors after CGo processing:
//     testdata/errors.go:102: cannot use 2 << 10 (untyped int constant 2048) as C.char value in variable declaration (overflows)
//     testdata/errors.go:105: unknown field z in struct literal
//     testdata/errors.go:108: undefined: C.SOME_CONST_1
//     testdata/errors.go:110: cannot use C.SOME_CONST_3 (untyped int constant 1234) as byte value in variable declaration (overflows)
//     testdata/errors.go:112: undefined: C.SOME_CONST_4
//     testdata/errors.go:114: undefined: C.SOME_CONST_b
//     testdata/errors.go:116: undefined: C.SOME_CONST_startspace
//     testdata/errors.go:119: undefined: C.SOME_PARAM_CONST_invalid
//     testdata/errors.go:122: undefined: C.add_toomuch
//     testdata/errors.go:123: undefined: C.add_toolittle

package main

import "syscall"
import "unsafe"

var _ unsafe.Pointer

//go:linkname C.CString runtime.cgo_CString
func C.CString(string) *C.char

//go:linkname C.GoString runtime.cgo_GoString
func C.GoString(*C.char) string

//go:linkname C.__GoStringN runtime.cgo_GoStringN
func C.__GoStringN(*C.char, uintptr) string

func C.GoStringN(cstr *C.char, length C.int) string {
	return C.__GoStringN(cstr, uintptr(length))
}

//go:linkname C.__GoBytes runtime.cgo_GoBytes
func C.__GoBytes(unsafe.Pointer, uintptr) []byte

func C.GoBytes(ptr unsafe.Pointer, length C.int) []byte {
	return C.__GoBytes(ptr, uintptr(length))
}

//go:linkname C.__CBytes runtime.cgo_CBytes
func C.__CBytes([]byte) unsafe.Pointer

func C.CBytes(b []byte) unsafe.Pointer {
	return C.__CBytes(b)
}

//go:linkname C.__get_errno_num runtime.cgo_errno
func C.__get_errno_num() uintptr

func C.__get_errno() error {
	return syscall.Errno(C.__get_errno_num())
}

type (
	C.char      uint8
	C.schar     int8
	C.uchar     uint8
	C.short     int16
	C.ushort    uint16
	C.int       int32
	C.uint      uint32
	C.long      int32
	C.ulong     uint32
	C.longlong  int64
	C.ulonglong uint64
)
type C.struct_point_t struct {
	x C.int
	y C.int
}
type C.point_t = C.struct_point_t

const C.SOME_CONST_3 = 1234
const C.SOME_PARAM_CONST_valid = 3 + 4
