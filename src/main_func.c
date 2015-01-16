#include "kern.h"
#include "context_switch.h"


int main(int argc, char *argv[]) {
  bwprintf(COM2, "Hellow\n\r");
  print_pc();
  bwprintf(COM2, "\n\rGoodbye\n\r");
  return 0;
}
