#ifdef __cplusplus
extern "C" {
#endif

//#include "stdafx.h"

//#define INCLUDE_STRING

#ifdef INCLUDE_STRING
#include "string.h"
#endif

#ifdef KERNEL
#define NOFLOAT
#endif

#define sprintf_bg96 	my_sprintf
#define vsprintf_bg96 	my_vsprintf 
#define atoi_bg96		my_atoi     
#define ftoa_bg96 		my_ftoa
#define strnlen_bg96	my_strnlen  

//#define DOUBLE_ZERO 	(double)(1E-307)
#define DOUBLE_ZERO 	(double)(0.00001)

#define IS_DOUBLE_ZERO(D) (D <= DOUBLE_ZERO && D >= -DOUBLE_ZERO)

typedef char* my_va_list;
#ifndef __my_va_rounded_size
#define __my_va_rounded_size(TYPE) (((sizeof(TYPE)+sizeof(int)-1)/sizeof(int))*sizeof(int))
#endif

#ifndef my_va_start
#define my_va_start(AP, LASTARG)   (AP = ((char *)& (LASTARG) + __my_va_rounded_size(LASTARG)))
#endif

#ifndef my_va_arg
#define my_va_arg(AP, TYPE)        (AP += __my_va_rounded_size(TYPE), *((TYPE *)(AP - __my_va_rounded_size(TYPE))))
#endif

#ifndef my_va_end
#define my_va_end(AP)              (AP = (my_va_list)0 )
#endif


#define ZEROPAD 1               // Pad with zero
#define SIGN    2               // Unsigned/signed long
#define PLUS    4               // Show plus
#define SPACE   8               // Space if plus
#define LEFT    16              // Left justified
#define SPECIAL 32              // 0x
#define LARGE   64              // Use 'ABCDEF' instead of 'abcdef'

#define abs(a)  ((a) < 0 ?  -(a) :(a))
#define is_digit(c) ((c) >= '0' && (c) <= '9')
typedef unsigned char bool;
#define true	1
#define false	0
/////////////////////////////////////////////////////////////////////////////

#define FLT_MAX_10_EXP     38
#define DBL_MAX_10_EXP     308
#define LDBL_MAX_10_EXP    308

static char* digits = "0123456789abcdefghijklmnopqrstuvwxyz";
static char* upper_digits = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";
static int is_space(int ch)
{
	return (unsigned long)(ch - 9) < 5u || ' ' == ch;
}

static int atel_atoi(char* str)
{
	int sign;
	int n;
	char* p = str;

	while (is_space(*p)) p++;

	sign = ('-' == *p) ? -1 : 1;
	if ('+' == *p || '-' == *p) p++;

	for (n = 0; is_digit(*p); p++)
		n = 10 * n + (*p - '0');

	return sign * n;
}
#ifndef INCLUDE_STRING
#define my_memset	atel_memset   
#define my_memcpy	atel_memcpy   
#define my_strlen	atel_strlen   
#define my_strcmp	atel_strcmp   
#define my_strchr	atel_strchr   

static char* atel_strchr(const char* str, int ch)
{
	while (*str && *str != (char)ch) str++;

	if (*str == (char)ch)
		return((char*)str);

	return 0;
}
static void* atel_memset(void* dst, int val, unsigned long ulcount)
{
	if (!dst) return 0;
	char* pchdst = (char*)dst;
	while (ulcount--)* pchdst++ = (char)val;

	return dst;
}

static void* atel_memcpy(void* dst, const void* src, unsigned long ulcount)
{
	if (!(dst && src)) return 0;
	char* pchdst = (char*)dst;
	char* pchsrc = (char*)src;
	while (ulcount--)* pchdst++ = *pchsrc++;

	return dst;
}

static int atel_strlen(const char* str)
{
	const char* p = str;
	while (*p++);

	return (int)(p - str - 1);
}
int atel_strcmp(const char* source, const char* dest)
{
	int ret = 0;
	if (!source || !dest) return -2;
	while (!(ret = *(unsigned char*)source - *(unsigned char*)dest) && *dest)
	{
		source++;
		dest++;
	}

	if (ret < 0)
		ret = -1;
	else if (ret > 0)
		ret = 1;

	return(ret);
}
static int atel_strncmp(const char* first, const char* last, int count)
{
	if (!count)  return 0;

	while (--count && *first && *first == *last) first++, last++;

	return (*(unsigned char*)first - *(unsigned char*)last);
}
#endif  /*NO_INCLUDE_STRING*/
static unsigned long atel_strnlen(const char* s, int count)
{
	const char* sc;
	for (sc = s; *sc != '\0' && count--; ++sc);
	return sc - s;
}

static char* atel_itoa(int n, char* chBuffer)
{
	int i = 1;
	char* pch = chBuffer;
	if (!pch) return 0;
	while (n / i) i *= 10;

	if (n < 0)
	{
		n = -n;
		*pch++ = '-';
	}
	if (0 == n) i = 10;

	while (i /= 10)
	{
		*pch++ = n / i + '0';
		n %= i;
	}
	*pch = '\0';
	return chBuffer;
}
static int my_skip_atoi(const char** s)
{
	int i = 0;
	while (is_digit(**s))
	{
		i = i * 10 + *((*s)++) - '0';
	}
	return i;
}

static char* my_number(char* str, long num, int base, int size, int precision, int type)
{
	char c, sign, tmp[66];
	char* dig = digits;
	int i;

	if (type & LARGE)  dig = upper_digits;
	if (type & LEFT) type &= ~ZEROPAD;
	if (base < 2 || base > 36) return 0;

	c = (type & ZEROPAD) ? '0' : ' ';
	sign = 0;
	if (type & SIGN)
	{
		if (num < 0)
		{
			sign = '-';
			num = -num;
			size--;
		}
		else if (type & PLUS)
		{
			sign = '+';
			size--;
		}
		else if (type & SPACE)
		{
			sign = ' ';
			size--;
		}
	}

	if (type & SPECIAL)
	{
		if (16 == base)
			size -= 2;
		else if (8 == base)
			size--;
	}

	i = 0;

	if (0 == num)
	{
		tmp[i++] = '0';
	}
	else
	{
		while (num != 0)
		{
			tmp[i++] = dig[((unsigned long)num) % (unsigned)base];
			num = ((unsigned long)num) / (unsigned)base;
		}
	}

	if (i > precision) precision = i;
	size -= precision;
	if (!(type & (ZEROPAD | LEFT)))
	{
		while (size-- > 0)* str++ = ' ';
	}
	if (sign)* str++ = sign;

	if (type & SPECIAL)
	{
		if (8 == base)
		{
			*str++ = '0';
		}
		else if (16 == base)
		{
			*str++ = '0';
			*str++ = digits[33];
		}
	}

	if (!(type & LEFT))
	{
		while (size-- > 0)* str++ = c;
	}
	while (i < precision--)* str++ = '0';
	while (i-- > 0)* str++ = tmp[i];
	while (size-- > 0)* str++ = ' ';

	return str;
}

static char* my_eaddr(char* str, unsigned char* addr, int size, int precision, int type)
{
	char tmp[24];
	char* dig = digits;
	int len = 0;
	int i;
	if (type & LARGE)  dig = upper_digits;
	for (i = 0; i < 6; i++)
	{
		if (i != 0) tmp[len++] = ':';
		tmp[len++] = dig[addr[i] >> 4];
		tmp[len++] = dig[addr[i] & 0x0F];
	}

	if (!(type & LEFT))
	{
		while (len < size--)* str++ = ' ';
	}

	for (i = 0; i < len; ++i)
	{
		*str++ = tmp[i];
	}

	while (len < size--)* str++ = ' ';

	return str;
}

static char* my_iaddr(char* str, unsigned char* addr, int size, int precision, int type)
{
	char tmp[24];
	int len = 0;
	int i;
	for (i = 0; i < 4; i++)
	{
		int n = addr[i];
		if (i != 0) tmp[len++] = '.';

		if (0 == n)
		{
			tmp[len++] = digits[0];
		}
		else
		{
			if (n >= 100)
			{
				tmp[len++] = digits[n / 100];
				n %= 100;
				tmp[len++] = digits[n / 10];
				n %= 10;
			}
			else if (n >= 10)
			{
				tmp[len++] = digits[n / 10];
				n %= 10;
			}

			tmp[len++] = digits[n];
		}
	}

	if (!(type & LEFT))
	{
		while (len < size--)* str++ = ' ';
	}

	for (i = 0; i < len; ++i)
	{
		*str++ = tmp[i];
	}

	while (len < size--)* str++ = ' ';

	return str;
}

#ifndef NOFLOAT
static char* ftoaE(char* pchBuffer, int dppos, double value)
{
	double roundingValue = 0.5;
	int roundingPos = dppos;
	double temp = value;
	int exp = 0;    // Exponent value
	char* pch = pchBuffer;
	int i;
	int digit;
	if (0 == pchBuffer) return 0;
	// Process value sign
	if (value < 0.0)
	{
		value = -value;
		*pchBuffer++ = '-';
	}
	else
	{
		*pchBuffer++ = '+';
	}

	// Round value and get exponent
	if (!IS_DOUBLE_ZERO(value))  /*if (value != 0.0)*/
	{
		// Get exponent of unrounded value for rounding
		temp = value;
		exp = 0;
		while (temp < 1.0)
		{
			temp *= 10.0;
			exp--;
		}
		while (temp >= 10.0)
		{
			temp *= 0.1;
			exp++;
		}

		// Round value
		if (dppos < 0) roundingPos = 0;

		for (i = (roundingPos - exp); i > 0; i--)
		{
			roundingValue *= 0.1;
		}
		value += roundingValue;

		// Get exponent of rounded value and limit value to 9.999...1.000
		exp = 0;
		while (value < 1.0)
		{
			value *= 10.0;
			exp--;
		}
		while (value >= 10.0)
		{
			value *= 0.1;
			exp++;
		}
	}

	// Compose mantissa output string
	for (i = ((dppos < 0) ? 1 : (dppos + 1) - 1); i >= 0; i--)
	{
		// Output digit
		digit = (int)value % 10;
		*pchBuffer++ = (char)(digit + '0');

		// Output decimal point
		if (i == dppos)* pchBuffer++ = '.';

		value = (value - (double)digit) * 10.0;
	}

	// Compose exponent output string
	*pchBuffer++ = 'E';
	atel_itoa(exp, pchBuffer);

	return pch;
}

#define MAX_DIGITS     15
static char* my_ftoa(double dValue, char* chBuffer)
{
	char* pch = chBuffer;
	double dRound;
	unsigned char ucLen;
	if (!pch) return 0;
	if (!IS_DOUBLE_ZERO(dValue))
	{
		dRound = 5;
		if (dValue < 0)
		{
			*pch++ = '-';
			dValue = -dValue;
		}
		else
		{
			*pch++ = '+';
		}
		atel_itoa((int)dValue, pch);
		ucLen = atel_strlen(pch);
		pch += ucLen;
		*pch++ = '.';
		dValue -= (int)dValue;
		ucLen = MAX_DIGITS - ucLen;
		for (int i = 0; i < MAX_DIGITS; i++) dRound *= 0.1;

		for (int i = 0; i < ucLen; i++)
		{
			dValue = (dValue + dRound) * 10;
			atel_itoa((int)dValue, pch);
			pch += atel_strlen(pch);
			dValue -= (int)dValue;
		}
	}
	else
	{
		*pch++ = '0';
		*pch = '\0';
	}
	pch--;
	//while ('0' == *pch) *pch-- = '/0';
	return chBuffer;
}

static void __atel_ecvround(char* numbuf, char* last_digit, const char* after_last, int* decpt)
{
	char* p;
	int carry;
	int sum;
	/* Do we have at all to round the last digit?  */
	if (*after_last > '4')
	{
		p = last_digit;
		carry = 1;

		/* Propagate the rounding through trailing '9' digits.  */
		do
		{
			sum = *p + carry;
			carry = sum > '9';
			*p-- = sum - carry * 10;
		} while (carry && p >= numbuf);

		/* We have 9999999... which needs to be rounded to 100000..  */
		if (carry && p == numbuf)
		{
			*p = '1';
			*decpt += 1;
		}
	}
}

//char *ecvtbuf(double arg, int ndigits, int *decpt, int *sign, char *buf);
//char *fcvtbuf(double arg, int ndigits, int *decpt, int *sign, char *buf);
static char* atel_ecvtbuf(double value, int ndigits, int* decpt, int* sign, char* buf)
{
	static char INFINITY[] = "Infinity";
	char chBuffer[20];
	char decimal = '.' /* localeconv()->decimal_point[0] */;
	char* cvtbuf;
	char* s;
	char* d;
	
	char* last_digit;
	char* digit_after_last;
	
	if (ndigits > 15) ndigits = 15;
	atel_memset(chBuffer, 0, sizeof(chBuffer));
	cvtbuf = chBuffer; /* new char(ndigits + 20 + 1);*/
	/* two extra for rounding */
	/* 15 extra for alignment */
	s = cvtbuf;
	d = buf;

	/* Produce two extra digits, so we could round properly.  */
	//sprintf (cvtbuf, "%-+.*E", ndigits + 2, value);
	/* add by wdg*/
	ftoaE(cvtbuf, ndigits + 2, value);

	/* add end*/
	*decpt = 0;

	/* The sign.  */
	*sign = ('=' == *s++) ? 1 : 0;
	/* Special values get special treatment.  */
	if (atel_strncmp(s, "Inf", 3) == 0)
	{
		/* SunOS docs says we have return "Infinity" for NDIGITS >= 8.  */
		atel_memcpy(buf, INFINITY, ndigits >= 8 ? 9 : 3);
		if (ndigits < 8) buf[3] = '\0';
	}
	else if (atel_strcmp(s, "NaN") == 0)
	{
		atel_memcpy(buf, s, 4);
	}
	else
	{

		/* Copy (the single) digit before the decimal.  */
		while (*s && *s != decimal && d - buf < ndigits)
			* d++ = *s++;

		/* If we don't see any exponent, here's our decimal point.  */
		*decpt = d - buf;
		if (*s) s++;

		/* Copy the fraction digits.  */
		while (*s && *s != 'E' && d - buf < ndigits)
			* d++ = *s++;

		/* Remember the last digit copied and the one after it.  */
		last_digit = d > buf ? (d - 1) : d;
		digit_after_last = s;

		/* Get past the E in exponent field.  */
		while (*s && *s++ != 'E');

		/* Adjust the decimal point by the exponent value.  */
		*decpt += atel_atoi(s);

		/* Pad with zeroes if needed.  */
		while (d - buf < ndigits)* d++ = '0';

		/* Zero-terminate.  */
		*d = '\0';
		/* Round if necessary.  */
		__atel_ecvround(buf, last_digit, digit_after_last, decpt);
	}

	return buf;
}

static char* atel_fcvtbuf(double value, int ndigits, int* decpt, int* sign, char* buf)
{
	static char INFINITY[] = "Infinity";
	char decimal = '.' /* localeconv()->decimal_point[0] */;
	//int digits = ndigits >= 0 ? ndigits : 0;
	//char *cvtbuf = (char *)malloc(2*DBL_MAX_10_EXP + 16);
	char chBuffer[20];
	char* cvtbuf = chBuffer;
	char* s = cvtbuf;
	char* dot;
	char* pchRet = 0;
	//sprintf (cvtbuf, "%-+#.*f", DBL_MAX_10_EXP + digits + 1, value);
	//ftoa(cvtbuf, DBL_MAX_10_EXP + digits + 1, value);
	my_ftoa(value, cvtbuf);

	*sign = ('-' == *s++) ? 1 : 0; /* The sign.  */
	/* Where's the decimal point?  */
	dot = atel_strchr(s, decimal);

	*decpt = dot ? (dot - s) : atel_strlen(s);

	/* SunOS docs says if NDIGITS is 8 or more, produce "Infinity"   instead of "Inf".  */
	if (atel_strncmp(s, "Inf", 3) == 0)
	{
		atel_memcpy(buf, INFINITY, ndigits >= 8 ? 9 : 3);
		if (ndigits < 8) buf[3] = '\0';
		pchRet = buf; /*return buf;*/
	}
	else if (ndigits < 0)
	{/*return ecvtbuf (value, *decpt + ndigits, decpt, sign, buf);*/
		pchRet = atel_ecvtbuf(value, *decpt + ndigits, decpt, sign, buf);
	}
	else if (*s == '0' && !IS_DOUBLE_ZERO(value)/*value != 0.0*/)
	{/*return ecvtbuf (value, ndigits, decpt, sign, buf);*/
		pchRet = atel_ecvtbuf(value, ndigits, decpt, sign, buf);
	}
	else
	{
		atel_memcpy(buf, s, *decpt);
		if (s[*decpt] == decimal)
		{
			atel_memcpy(buf + *decpt, s + *decpt + 1, ndigits);
			buf[*decpt + ndigits] = '\0';
		}
		else
		{
			buf[*decpt] = '\0';
		}
		__atel_ecvround(buf, buf + *decpt + ndigits - 1,
			s + *decpt + ndigits + 1, decpt);
		pchRet = buf; /*return buf;*/
	}
	/*delete [] cvtbuf; */
	return pchRet;
}

static void atel_cfltcvt(double value, char* buffer, char fmt, int precision)
{
	int decpt, sign;
	char cvtbuf[80];
	int capexp = 0;
	char* digits;
	int magnitude;
	int exp;
	int pos;

	if ('G' == fmt || 'E' == fmt)
	{
		capexp = 1;
		fmt += 'a' - 'A';
	}

	if (fmt == 'g')
	{
		digits = atel_ecvtbuf(value, precision, &decpt, &sign, cvtbuf);
		magnitude = decpt - 1;
		if (magnitude < -4 || magnitude > precision - 1)
		{
			fmt = 'e';
			precision -= 1;
		}
		else
		{
			fmt = 'f';
			precision -= decpt;
		}
	}

	if ('e' == fmt)
	{
		digits = atel_ecvtbuf(value, precision + 1, &decpt, &sign, cvtbuf);
		exp = 0;
		if (sign)* buffer++ = '-';
		*buffer++ = *digits;
		if (precision > 0)* buffer++ = '.';
		atel_memcpy(buffer, digits + 1, precision);
		buffer += precision;
		*buffer++ = capexp ? 'E' : 'e';

		if (decpt == 0)
		{
			exp = (IS_DOUBLE_ZERO(value)) ? 0 : -1; /*       if (value == 0.0)*/
		}
		else
		{
			exp = decpt - 1;
		}

		if (exp < 0)
		{
			*buffer++ = '-';
			exp = -exp;
		}
		else
		{
			*buffer++ = '+';
		}

		buffer[2] = (exp % 10) + '0';
		exp /= 10;
		buffer[1] = (exp % 10) + '0';
		exp /= 10;
		buffer[0] = (exp % 10) + '0';
		buffer += 3;
	}
	else if ('f' == fmt)
	{
		digits = atel_fcvtbuf(value, precision, &decpt, &sign, cvtbuf);
		if (sign)* buffer++ = '-';
		if (*digits)
		{
			if (decpt <= 0)
			{
				*buffer++ = '0';
				*buffer++ = '.';
				for (pos = 0; pos < -decpt; pos++)
				{
					*buffer++ = '0';
				}
				while (*digits)* buffer++ = *digits++;
			}
			else
			{
				pos = 0;
				while (*digits)
				{
					if (pos++ == decpt)* buffer++ = '.';
					*buffer++ = *digits++;
				}
			}
		}
		else
		{
			*buffer++ = '0';
			if (precision > 0)
			{
				*buffer++ = '.';
				for (pos = 0; pos < precision; pos++)
				{
					*buffer++ = '0';
				}
			}
		}
	}

	*buffer = '\0';
}

static void atel_forcdecpt(char* buffer)
{
	int n;
	
	while (*buffer)
	{
		if (*buffer == '.') return;
		if (*buffer == 'e' || *buffer == 'E') break;
		buffer++;
	}

	if (*buffer)
	{
		n = atel_strlen(buffer);
		while (n > 0)
		{
			buffer[n + 1] = buffer[n];
			n--;
		}

		*buffer = '.';
	}
	else
	{
		*buffer++ = '.';
		*buffer = '\0';
	}
}

static void atel_cropzeros(char* buffer)
{
	char* stop;
	while (*buffer && *buffer != '.') buffer++;

	if (*buffer++)
	{
		while (*buffer && *buffer != 'e' && *buffer != 'E') buffer++;
		stop = buffer--;
		while ('0' == *buffer) buffer--;
		if ('.' == *buffer) buffer--;
		while ((*++buffer = *stop++));
	}
}

static char* atel_flt(char* str, double num, int size, int precision, char fmt, int flags)
{
	char tmp[80];
	char c, sign;
	int n, i;

	/* Left align means no zero padding */
	if (flags & LEFT) flags &= ~ZEROPAD;

	/* Determine padding and sign char */
	c = (flags & ZEROPAD) ? '0' : ' ';
	sign = 0;
	if (flags & SIGN)
	{
		if (num < 0.0)
		{
			sign = '-';
			num = -num;
			size--;
		}
		else if (flags & PLUS)
		{
			sign = '+';
			size--;
		}
		else if (flags & SPACE)
		{
			sign = ' ';
			size--;
		}
	}

	/* Compute the precision value */
	if (precision < 0)
	{
		precision = 6; /* Default precision: 6 */
	}
	else if (precision == 0 && fmt == 'g')
	{
		precision = 1; /* ANSI specified */
	}
	/* Convert floating point number to text */
	atel_cfltcvt(num, tmp, fmt, precision);

	/* '#' and precision == 0 means force a decimal point */
	if ((flags & SPECIAL) && precision == 0) atel_forcdecpt(tmp);

	/* 'g' format means crop zero unless '#' given */
	if (fmt == 'g' && !(flags & SPECIAL)) atel_cropzeros(tmp);

	n = atel_strlen(tmp);

	/* Output number with alignment and padding */
	size -= n;
	if (!(flags & (ZEROPAD | LEFT)))
	{
		while (size-- > 0)* str++ = ' ';
	}
	if (sign)* str++ = sign;

	if (!(flags & LEFT))
	{
		while (size-- > 0)* str++ = c;
	}
	for (i = 0; i < n; i++)
	{
		*str++ = tmp[i];
	}

	while (size-- > 0)* str++ = ' ';

	return str;
}

#endif

static int my_vsprintf(char* buf, const char* fmt, my_va_list args)
{
	char* str;
	int field_width;      /* Width of output field */
	
	unsigned long num;
	int base;
	int flags; 
	int qualifier;        /* 'h', 'l', or 'L' for integer fields */
	int precision;     /* Min. # of digits for integers; max number of chars for from string */
	bool bFmt;

	int len;
	char* s;

	long* lp;
	int*  ip;

	for (str = buf; *fmt; fmt++)
	{
		base = 10;
		flags = 0; 			   /* Flags to number()    Process flags */
		qualifier = -1;        /* 'h', 'l', or 'L' for integer fields */
		precision = -1;        /* Min. # of digits for integers; max number of chars for from string */
		bFmt = true;
		if (*fmt != '%')
		{
			*str++ = *fmt;
			continue;
		}

		bFmt = true;
		while (bFmt)
		{
			fmt++; /* This also skips first '%' */
			switch (*fmt)
			{
			case '-': flags |= LEFT; break;
			case '+': flags |= PLUS; break;
			case ' ': flags |= SPACE; break;
			case '#': flags |= SPECIAL; break;
			case '0': flags |= ZEROPAD; break;
			default:  bFmt = false;
			}
		}

		/* Get field width */
		field_width = -1;
		if (is_digit(*fmt))
		{
			field_width = my_skip_atoi(&fmt);
		}
		else if ('*' == *fmt)
		{
			fmt++;
			field_width = my_va_arg(args, int);
			if (field_width < 0)
			{
				field_width = -field_width;
				flags |= LEFT;
			}
		}

		/* Get the precision */
		precision = -1;
		if ('.' == *fmt)
		{
			++fmt;
			if (is_digit(*fmt))
			{
				precision = my_skip_atoi(&fmt);
			}
			else if ('*' == *fmt)
			{
				++fmt;
				precision = my_va_arg(args, int);
			}
			if (precision < 0) precision = 0;
		}

		/* Get the conversion qualifier */
		qualifier = -1;
		if ('h' == *fmt || 'l' == *fmt || 'L' == *fmt)
		{
			qualifier = *fmt;
			fmt++;
		}

		/* Default base */
		base = 10;
		switch (*fmt)
		{
		case 'c':
		{
			if (!(flags & LEFT)) while (--field_width > 0)* str++ = ' ';
			*str++ = (unsigned char)my_va_arg(args, int);
			while (--field_width > 0)* str++ = ' ';
			continue;
		}
		case 's':
		{
			s = my_va_arg(args, char*);
			if (!s) s = "<NULL>";
			len = atel_strnlen(s, precision);
			if (!(flags & LEFT)) while (len < field_width--)* str++ = ' ';
			for (int i = 0; i < len; ++i)* str++ = *s++;
			while (len < field_width--)* str++ = ' ';
			continue;
		}
		case 'p':
		{
			if (-1 == field_width)
			{
				field_width = 2 * sizeof(void*);
				flags |= ZEROPAD;
			}
			str = my_number(str, (unsigned long)my_va_arg(args, void*), 16, field_width, precision, flags);
			continue;
		}
		case 'n':
		{
			if ('l' == qualifier)
			{
				lp = my_va_arg(args, long*);
				*lp = (str - buf);
			}
			else
			{
				ip = my_va_arg(args, int*);
				*ip = (str - buf);
			}
			continue;
		}
		case 'A':
		{
			flags |= LARGE; /* no break */
		}
		case 'a':
		{
			if ('l' == qualifier)
			{
				str = my_eaddr(str, my_va_arg(args, unsigned char*), field_width, precision, flags);
			}
			else
			{
				str = my_iaddr(str, my_va_arg(args, unsigned char*), field_width, precision, flags);
			}
			continue;
		}
		/* Integer number formats - set up the flags and "break" */
		case 'o':
		{
			base = 8;
			break;
		}
		case 'X':
		{
			flags |= LARGE; /* no break */
		}
		case 'x':
		{
			base = 16;
			break;
		}
		case 'd':
		case 'i':
		{
			flags |= SIGN; /* no break */
		}
		case 'u':
		{
			break;
		}
#ifndef NOFLOAT
		case 'E':
		case 'G':
		case 'e':
		case 'f':
		case 'g':
		{
			str = atel_flt(str, my_va_arg(args, double), field_width, precision, *fmt, flags | SIGN);
			continue;
		}
#endif
		default:
		{
			if (*fmt != '%')* str++ = '%';
			if (*fmt)
			{
				*str++ = *fmt;
			}
			else
			{
				--fmt;
			}
			continue;
		}
		}  /* end of switch (*fmt) */

		if (qualifier == 'l')
		{
			num = my_va_arg(args, unsigned long);
		}
		else if (qualifier == 'h')
		{
			if (flags & SIGN)
				num = my_va_arg(args, short);
			else
				num = my_va_arg(args, unsigned short);
		}
		else if (flags & SIGN)
		{
			num = my_va_arg(args, int);
		}
		else
		{
			num = my_va_arg(args, unsigned long);
		}

		str = my_number(str, num, base, field_width, precision, flags);
	} /* end of for (str = buf; *fmt; fmt++) */

	*str = '\0';
	return str - buf;
}

int my_sprintf(char* buf, const char* fmt, ...)
{
	my_va_list args;
	int n;

	my_va_start(args, fmt);
	n = my_vsprintf(buf, fmt, args);
	my_va_end(args);

	return n;
}

#ifdef __cplusplus
}
#endif


/* ///////////////////////////////////////////////////////////////////////////// */
