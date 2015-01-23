.file	"context_switch.c"
.text

.align 2
.global init_regs
.type init_regs, %function
init_regs:
  mov ip, sp
  stmfd sp!, {fp, ip, lr}
  mov r0, #0
  mov r1, #0
  mov r2, #0
  mov r3, #0
  mov r4, #0
  mov r5, #0
  mov r6, #0
  mov r7, #0
  mov r8, #0
  mov r9, #0
  @mov r10, #0 @ apparently we cannot clear this register @,@? 
  ldmfd sp, {fp, sp, pc}

.align 2
.global init_kernelentry
.type init_kernelentry, %function
init_kernelentry:
  mov ip, sp
  stmfd sp!, {fp, ip, lr}
  ldr r1, =kernelentry
  add r1, r1, #0x218000 
  mov r2, #0x28
  str r1, [r2]
  ldmfd sp, {fp, sp, pc}

.align 2
.global kernel_exit
.type kernel_exit, %function
@TODO: find out args etc settings

kernel_exit:
  @ 1. push kregs onto stack
  mov ip, sp
  stmfd sp!, {r3-r12, r14}
  @ Set spsr to saved value
  msr spsr, r2
  @ 2. switch to system mode
  msr cpsr_c, #0xdf
  @ 3. get retval, sp from TD
  mov sp, r1
  @ load the pc into r1
  ldmfd sp!, {r1}
  @ r0 is already retval
  @ 4. pop the registers of the active task 
  ldmfd sp!, {r4-r12, r14}
  @ 6. return to svc state
  msr cpsr_c, #0xd3
  @ 8. install the pc of the active task and change modes
  movs pc, r1
  @ldmfd sp, {r3-r12, pc}


@.align 2
@.global kernel_entry
@.type kernel_entry, %function
kernelentry:
  @ 3. change to system state, sp lr are in user
  msr cpsr_c, #0xdf
  @ 5. store user's registers values onto user stack
  stmfd sp!, {r4-r12, r14}
  @ 6. save active user task's sp in r1
  mov r1, sp
  @ 7. back to service state
  msr cpsr_c, #0xd3
  @ Calculate address of SWI instruction and load it into r0.
  ldr r0, [lr,#-4]
  @ Mask off top 8 bits of instruction to give SWI number.
  bic r0, r0,#0xFF000000
  @ save user's next pc to stack which is stored in lr_svc, set by swi
  stmfd r1!, {lr}
  @ 8. getting spsr, user's values were set by swi
  mrs r2, spsr
  @ 9. pop kernel registers from stack, jump to kernel's lr instruction
  ldmfd sp!, {r3-r12, r14}
  mov sp, ip
  mov pc, lr
  @ r0: request_type, r1: user sp, r2: user spsr, arguments on user stack


