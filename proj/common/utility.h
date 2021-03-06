
#pragma once

#define abs(a)   (((a)>0)?((a)):(-(a)))

#define cat2(i,j)       i##j
#define cat3(i,j,k)     i##j##k

#ifndef min
#define min(a,b)	((a) < (b) ? (a) : (b))
#endif

#ifndef min2
#define min2(a,b)	((a) < (b) ? (a) : (b))
#endif

#ifndef min3
#define min3(a,b,c)	min2(min2(a, b), c)
#endif

#ifndef max2
#define max2(a,b)	((a) > (b) ? (a): (b))
#endif

#ifndef max3
#define max3(a,b,c)	max2(max2(a, b), c)
#endif

#define OFFSETOF(s, m) 			((unsigned int) &((s *)0)->m)
#define ROUND_INT(x, r)			(((x) + (r) - 1) / (r) * (r))
#define ROUND_TO_POW2(x, r)		(((x) + (r) - 1) & ~((r) - 1))

//  direct memory access 
#define U8_GET(addr)			(*(volatile unsigned char  *)(addr))
#define U16_GET(addr)			(*(volatile unsigned short *)(addr))
#define U32_GET(addr)			(*(volatile unsigned int  *)(addr))

#define U8_SET(addr, v)			(*(volatile unsigned char  *)(addr) = (unsigned char)(v))
#define U16_SET(addr, v)		(*(volatile unsigned short *)(addr) = (unsigned short)(v))
#define U32_SET(addr, v)		(*(volatile unsigned int  *)(addr) = (v))

#define U8_INC(addr)			U8_GET(addr) += 1
#define U16_INC(addr)			U16_GET(addr) += 1
#define U32_INC(addr)			U32_GET(addr) += 1

#define U8_DEC(addr)			U8_GET(addr) -= 1
#define U16_DEC(addr)			U16_GET(addr) -= 1
#define U32_DEC(addr)			U32_GET(addr) -= 1

#define U8_CPY(addr1,addr2)		U8_SET(addr1, U8_GET(addr2))
#define U16_CPY(addr1,addr2)	U16_SET(addr1, U16_GET(addr2))
#define U32_CPY(addr1,addr2)	U32_SET(addr1, U32_GET(addr2))

#define MAKE_U16(h,l) 			((unsigned short)(((h) << 8) | (l)))
#define MAKE_U32(a,b,c,d)		((unsigned short)(((a) << 24) | ((b) << 16) ((c) << 8) | (d)))

#define BOUND(x, l, m)			((x) < (l) ? (l) : ((x) > (m) ? (m) : (x)))
#define SET_BOUND(x, l, m)		((x) = BOUND(x, l, m))
#define BOUND_INC(x, m)			do{++(x); (x) = (x) < (m) ? (x) :0;} while(0)
#define BOUND_INC_POW2(x, m)	do{								\
									STATIC_ASSERT_POW2(m);		\
									(x) = ((x)+1) & (m-1);		\
								}while(0)

#define IS_POWER_OF_2(x)		(!(x & (x-1)))
#define IS_LITTLE_ENDIAN 		(*(unsigned short*)"\0\xff" > 0x100) 

#define IMPLIES(x, y) 			(!(x) || (y))

// x > y ? 1 : (x ==y : 0 ? -1)
#define COMPARE(x, y) 			(((x) > (y)) - ((x) < (y)))
#define SIGN(x) 				COMPARE(x, 0)

// better than xor swap:  http://stackoverflow.com/questions/3912699/why-swap-with-xor-works-fine-in-c-but-in-java-doesnt-some-puzzle
#define SWAP(x, y, T) 			do { T tmp = (x); (x) = (y); (y) = tmp; } while(0)
#define SORT2(a, b, T) 			do { if ((a) > (b)) SWAP((a), (b), T); } while (0)

#define foreach(i, n) 			for(int i = 0; i < (n); ++i)
#define foreach_range(i, s, e) 	for(int i = (s); i < (e); ++i)
#define foreach_arr(i, arr) 	for(int i = 0; i < ARRAY_SIZE(arr); ++i)
//  round robbin foreach,   ??????????????????????????,  h ??????????????????????????????????????????????h ????????n !!!
#define foreach_hint(i, n, h) 	for(int i = 0, ++h, h=h<n?h:0; i < n; ++h, h=h<n?h:0)

#define ARRAY_SIZE(a) 			(sizeof(a) / sizeof(*a))

#define everyN(i, n) 			++(i); (i)=((i) < N ? (i) : 0); if(0 == (i))

