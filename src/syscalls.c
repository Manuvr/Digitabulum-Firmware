#ifdef __cplusplus
extern "C" {
#endif

#include <inttypes.h>
#include <errno.h>
#include <sys/times.h>
#include <sys/unistd.h>

#include <sys/stat.h>
#include "stm32f7xx.h"
//#include "stm32f7xx_conf.h"
//#include "usbd_cdc_vcp.h"



/*
 getpid
 Process-ID; this is sometimes used to generate strings unlikely to conflict with other processes. Minimal implementation, for a system without processes:
*/
int _getpid() {                    return 1;  }
int _close(int file) {                return 0;  }
int _fstat(int file, struct stat *st) {        return 0;  }
int _isatty(int file) {                return 1;  }
int _lseek(int file, int ptr, int dir) {        return 0;  }
int _open(const char *name, int flags, int mode) {    return -1;  }

/*
 execve
 Transfer control to a new process. Minimal implementation (for a system without processes):
 */
int _execve(char *name, char **argv, char **env) {
    errno = ENOMEM;
    return -1;
}

/*
 fork
 Create a new process. Minimal implementation (for a system without processes):
*/
int _fork() {
    errno = EAGAIN;
    return -1;
}

int _read(int file, char *ptr, int len) {
  if (file != 0) {
    return 0;
  }

  // Use USB CDC Port for stdin
  //while(!VCP_get_char((uint8_t*)ptr)){};

  // Echo typed characters
  //VCP_put_char((uint8_t)*ptr);

  return 1;
}

/*
 kill
 Send a signal. Minimal implementation:
 */
int _kill(int pid, int sig) {
    errno = EINVAL;
    return (-1);
}


int _write(int file, char *ptr, int len) {
  //VCP_send_buffer((uint8_t*)ptr, len);
  return len;
}



/*
 sbrk
 Increase program data space.
 Malloc and related functions depend on this
 */
caddr_t _sbrk(int incr) {

    extern char _ebss; // Defined by the linker
    static char *heap_end;
    char *prev_heap_end;

    if (heap_end == 0) {
        heap_end = &_ebss;
    }
    prev_heap_end = heap_end;
    
    register uint32_t stack;
    __asm volatile ("MRS %0, msp\n" : "=r" (stack) );

    //if (heap_end + incr > (char*) stack) {
    //    _write (STDERR_FILENO, (char*) "Heap and stack collision\n", 25);
    //    errno = ENOMEM;
    //    return  (caddr_t) -1;
    //    //abort ();
    //}

    heap_end += incr;
    return (caddr_t) prev_heap_end;
}



/* Register name faking - works in collusion with the linker.  */
register char * stack_ptr asm ("sp");

caddr_t _sbrk_r (struct _reent *r, int incr) {
  extern char   end asm ("end"); /* Defined by the linker.  */
  static char * heap_end;
  char *        prev_heap_end;

  if (heap_end == NULL)
    heap_end = & end;

  prev_heap_end = heap_end;

  //if (heap_end + incr > stack_ptr) {
  //  //errno = ENOMEM;
  //  return (caddr_t) -1;
  //}

  heap_end += incr;

  return (caddr_t) prev_heap_end;
}


#ifdef __cplusplus
}
#endif

