#ifndef   _U_TYPES_H_
#define   _U_TYPES_H_       universal types, by weiching.lin

// unfixed size
//
typedef   unsigned long     UPtr;   // unsigned pointer, uintptr_t
typedef     signed long     IPtr;   //   signed pointer, ptrdiff_t
typedef   unsigned long     UCnt;   // unsigned count, size_t
typedef     signed long     ICnt;   //   signed count, ssize_t
typedef       long long     LOff;   // long offset, loff_t
typedef     signed long     IOff;   //      offset, off_t
typedef   unsigned long     Bool;   // unsigned boolean

// fixed size
//
typedef   unsigned char     U8;     // -128..127
typedef     signed char     I8;     // 0..255
typedef   unsigned short    U16;    // -32768..32767
typedef     signed short    I16;    // 0..65535
typedef   unsigned int      U32;    // 0..4294967295
typedef     signed int      I32;    // -2147483648..2147483647
//
typedef   float             F32;    // 4 byte real
typedef   double            F64;    // 8 byte real

// boolean type
//
#ifndef   BOOL
#define   BOOL   			unsigned int
#endif

// byte order, little endian
//
#define   BYTE_1ST(value)   ( 0x00FF & ((value)      ))
#define   BYTE_2ND(value)   ( 0x00FF & ((value) >>  8))
#define   BYTE_3RD(value)   ( 0x00FF & ((value) >> 16))
#define   BYTE_4TH(value)   ( 0x00FF & ((value) >> 24))


#endif // _U_TYPES_H_
