
#include "qst_log.h"

#if defined(QST_LOG_USE_CUSTOM_FUNC)
#include "bsp_usart.h"
#define qst_log_send_ch(ch)		usart_send_ch(ch)

static uint8_t itoa10(uint8_t *buf, int32_t i)
{
     uint32_t rem;
     uint8_t *s,length=0;
	 uint8_t flag=0;

     s = buf;
     if (i == 0) 
     {
     	*s++ = '0'; 
     }
     else 
     { 
          if (i < 0) 
          {
	          *buf++ = '-';
			  flag = 1;
	          s = buf;
	          i = -i;
          }
          while (i) 
          {
	          ++length;
	          rem = i % 10;
	          *s++ = rem + '0';
	          i /= 10;
          }
          for(rem=0; ((unsigned char)rem)<length/2; rem++) 
          {
	          *(buf+length) = *(buf+((unsigned char)rem));
	          *(buf+((unsigned char)rem)) = *(buf+(length-((unsigned char)rem)-1));
	          *(buf+(length-((unsigned char)rem)-1)) = *(buf+length);
          }
     }
     *s=0;

	 return ((s-buf)+flag);
}

static void ftoa(uint8_t *buf, float i)
{
	int32_t i_data;
	uint8_t length=0;

	if((i>=0.000001f) && (i<=0.000001f))		
	{
	   *buf++ = '0'; 
	   *buf++ = '.'; 
	   *buf++ = '0'; 
	}
	else
	{
		if (i < 0) 
		{
			*buf++ = '-';
			i = -i;
		}
		i_data = (int32_t)i;
		length = itoa10(buf, i_data);
		buf += length;
		*buf++ = '.';
		i_data = (int32_t)((i-i_data)*10000);
		length = itoa10(buf, i_data);
		buf += length;
	}
	
	*buf=0;
}

static void qst_log_send_str(uint8_t *str)
{
	while(*str)
	{
		qst_log_send_ch(*str)
		str++;
	}
}

int qst_printf(const char *format, ...)
{
	int8_t *pc;
	int32_t value;
	float f_value;
	char c_value;
	uint8_t buf[50];
	
	va_list arg;
	va_start(arg, format);
	//buf[0]=va_arg(arg, char);
	while (*format)
	{
		int8_t ret = *format;
		if(ret == '%')
		{
			switch (*++format)
			{
			case 'c':
			{
				buf[0] = va_arg(arg, char);
				//putchar(ch);
				buf[1] = 0;
				qst_log_send_str(buf);
				break;
			}
			case 's':
			{
				pc = va_arg(arg, int8_t *);
				//while (*pc)
				//{
				//	putchar(*pc);
				//	pc++;
				//}
				qst_log_send_str((uint8_t*)pc);
				break;
			}
			case 'd':
			{
				value =	va_arg(arg, int32_t);
				itoa10(buf, value);
				qst_log_send_str(buf);
				break;
			}
			case 'l':
			{
				value =	va_arg(arg, int64_t);
				itoa10(buf, value);
				qst_log_send_str(buf);
				break;
			}
			case 'f':
			{
				f_value = va_arg(arg, float);
				ftoa(buf, f_value);
				qst_log_send_str(buf);
				break;
			}			
			//case 'x':
			//{
			//	value = va_arg(arg, int32_t);
			//	ByteToHexStr((unsigned char)value, buf);
			//	qst_log_send_str(buf);
			//	break;
			//}
			default:
				break;
			}
		}
		else
		{
			qst_log_send_ch(ret);
		}
		format++;
	}
	va_end(arg);
	
	return 1;
}
#endif

#if defined(QST_LOG_USE_KEIL_PRINTF)
#define ITM_PORT8(n)         (*(volatile unsigned char *)(0xe0000000 + 4*(n)))
#define ITM_PORT16(n)        (*(volatile unsigned short *)(0xe0000000 + 4*(n)))
#define ITM_PORT32(n)        (*(volatile unsigned long *)(0xe0000000 + 4*(n)))
#define DEMCR                (*(volatile unsigned long *)(0xE000EDFC))
#define TRCENA               0X01000000
#define KEIL_BUF_LEN    100
static char keil_buff[KEIL_BUF_LEN];

int keil_printf(char *fmt, ...)
{
    va_list argptr;
    int cnt, i;

    va_start(argptr, fmt);
    cnt = vsnprintf(keil_buff, KEIL_BUF_LEN, fmt, argptr);
    va_end(argptr);

    for(i=0; i<cnt; i++)
    {
        if(DEMCR & TRCENA)
        {
                while(ITM_PORT32(0) == 0);
                ITM_PORT8(0) = keil_buff[i];
        }
    }

    return cnt;
}
#endif

#if defined(QST_LOG_USE_SEGGER_RTT)
#define SEGGER_BUF_LEN    100
static char segger_buff[SEGGER_BUF_LEN]={0};

int segger_printf(char *fmt, ...)
{
    va_list argptr;
    int cnt, i;

    va_start(argptr, fmt);
    cnt = vsnprintf(segger_buff, SEGGER_BUF_LEN, fmt, argptr);
    va_end(argptr);

	SEGGER_RTT_TerminalOut(0, segger_buff);

    return cnt;
}

#endif


