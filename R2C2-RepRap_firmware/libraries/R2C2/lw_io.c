/* Copyright (c) 2012 Bob Cousins bobcousins42@googlemail.com       */
/* Copyright (C) 2009-2010 Michael Moon aka Triffid_Hunter   */
/* Copyright (c) 2011 Jorge Pinto - casainho@gmail.com       */
/* All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.
   * Neither the name of the copyright holders nor the names of
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
*/

#include <string.h>

#include "lw_io.h"

#include "lw_syscalls.h"

#include "sermsg.h"


// Global FILE table

LW_FILE file_table [MAX_FILES] = 
{
    {.handle=0, .dev_major = DEV_STDIN,  .dev_minor = 0},
    {.handle=1, .dev_major = DEV_STDOUT, .dev_minor = 0},
    {.handle=2, .dev_major = DEV_STDERR, .dev_minor = 0},
    {.handle=3, .dev_major = DEV_DBGOUT, .dev_minor = 0},
};


// predefined file handles
LW_FILE *stdin  = &file_table[0];
LW_FILE *stdout = &file_table[1];
LW_FILE *stderr = &file_table[2];

LW_FILE *dbgout = &file_table[3];

// ----------------------------------------------------------------
// functions taking FILE parameter
// ----------------------------------------------------------------

int lw_fputs(const char *s, LW_FILE *f)
{
  return _write (f->handle, s, strlen (s));
}

int lw_fputc(int c, LW_FILE *f)
{
  return _write (f->handle, (char *)&c, 1);
}

int lw_putc(int c, LW_FILE *f)
{
  lw_fputc (c, f);
}

int lw_dbg_printf (const char *format, ...)
{
  // TODO
}

// was sersendf
int lw_printf (const char *format, ...)
{
  va_list args;
  va_start(args, format);

  unsigned int i = 0;
  unsigned char c, j = 0;

  while ((c = format[i++])) {
    if (j) {
      switch(c) {
        case 'l':
          j = 4;
          break;
        case 'u':
          if (j == 4)
            serwrite_uint32(va_arg(args, unsigned int));
          else
            serwrite_uint16(va_arg(args, unsigned int));
          j = 0;
          break;
        case 'd':
          if (j == 4)
            serwrite_int32(va_arg(args, int));
          else
            serwrite_int16(va_arg(args, int));
          j = 0;
          break;

          /* print a double in normal notation */
          case 'g':
            serwrite_double(va_arg(args, double));
            j = 0;
            break;

          case 'p':
          case 'x':
            lw_puts("0x");
            if (j == 4)
              serwrite_hex32(va_arg(args, unsigned int));
            else
              serwrite_hex16(va_arg(args, unsigned int));
            j = 0;
            break;
          case 'c':
            lw_putchar(va_arg(args, unsigned int));
            j = 0;
            break;
          case 's':
            lw_puts(va_arg(args, char *));
            j = 0;
            break;

          /* escape % char */
          case '%':
            lw_putchar('%');
            j = 0;
            break;

          default:
            j = 0;
            break;
        }
      }
      else {
        if (c == '%') {
          j = 2;
        }
      else {
        lw_putchar(c);
      }
    }
  }
  va_end(args);
}

// ----------------------------------------------------------------
// functions working on predefined streams: STDIN, STDOUT, STDERR
// ----------------------------------------------------------------


int  lw_puts(const char *s)
{
  return _write (stdout->handle, s, strlen (s));
}

int  lw_putchar(int c)
{
  return _write (stdout->handle, (char *) &c, 1);
}


// ----------------------------------------------------------------
// not yet implemented
// ----------------------------------------------------------------

#define __va_list va_list

int  getchar(void);
char *gets(char *__s);

int sprintf(char *__s, const char *__format, ...);
int snprintf(char *__s, size_t __n, const char *__format, ...);
int vsnprintf(char *__s, size_t __n, const char *__format, __va_list __arg);

int vprintf(const char *__format, __va_list __arg);
int vsprintf(char *__s, const char *__format, __va_list __arg);

int scanf(const char *__format, ...);
int sscanf(const char *__s, const char *__format, ...);
int vscanf(const char *__format, __va_list __arg);
int vsscanf(const char *__s, const char *__format, __va_list __arg);

void clearerr(LW_FILE *);
int fclose(LW_FILE *);
int feof(LW_FILE *);
int ferror(LW_FILE *);
int fflush(LW_FILE *);
int fgetc(LW_FILE *);
int fgetpos(LW_FILE *, fpos_t *);
char *fgets(char *, int, LW_FILE *);
LW_FILE *fopen(const char *, const char *);
int fprintf(LW_FILE *, const char *, ...);
size_t fread(void *, size_t, size_t, LW_FILE *);
LW_FILE *freopen(const char *, const char *, LW_FILE *);
int fscanf(LW_FILE *, const char *, ...);
int fseek(LW_FILE *, long, int);
int fsetpos(LW_FILE *, const fpos_t *); 
long ftell(LW_FILE *);
size_t fwrite(const void *, size_t, size_t, LW_FILE *);
int getc(LW_FILE *);

void perror(const char *);

int remove(const char *);
int rename(const char *, const char *);
void rewind(LW_FILE *);
void setbuf(LW_FILE *, char *);
int setvbuf(LW_FILE *, char *, int, size_t);
LW_FILE *tmpLW_FILE(void);
char * tmpnam(char *);
int ungetc(int, LW_FILE *);
int vfprintf(LW_FILE *, const char *, __va_list);
int vfscanf(LW_FILE *, const char *, __va_list);

// ----------------------------------------------------------------