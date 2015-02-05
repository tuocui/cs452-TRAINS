.file	"context_switch.c"
.text

.align 2
.global init_kernelentry
.type init_kernelentry, %function
init_kernelentry:
  mov ip, sp
  stmfd sp!, {fp, ip, lr}
  @ swi address
  ldr r1, =kernelentry
  add r1, r1, #0x218000 
  mov r2, #0x28
  str r1, [r2]
  @ hwi address
  ldr r1, =hwikernelentry
  add r1, r1, #0x218000
  mov r2, #0x38
  str r1, [r2]
  @
  ldmfd sp, {fp, sp, pc}

.align 2
.global kernel_exit
.type kernel_exit, %function
@ //TODO: find out args etc settings
kernel_exit:
  @ 1. push kregs onto stack
  mov ip, sp
  stmfd sp!, {r4-r12, r14}
  stmfd sp!, {r3}
  @ Set spsr to saved value
  msr spsr, r2
  @ load the user pc into r14 (svc_lr)
  ldmfd r1!, {r14}
  @ 2. switch to system mode
  msr cpsr_c, #0xdf
  @ 3. get retval, sp from TD
  mov sp, r1
  @ r0 is already retval
  @ 4. pop the registers of the active task 
  ldmfd sp!, {r1-r12, r14}
  @ 6. return to svc state
  msr cpsr_c, #0xd3
  @ 8. install the pc of the active task and change modes
  movs pc, r14


@ logistics: set hwi_flag on so in C we can detect hwi interrupt. Also need to
@            transfer lr_irq and spsr_irq to lr_svc and spsr_svc, we do this by
@            using scratch r0 and r1. However, we must persist their original 
@            values so we push them on and pop them off of the kernel stack.
@            Later on kernelentry will naively pick up these two values.
hwikernelentry:
  @testing user's r0
  @mov r1, r0
  @mov r0, #1
  @bl bwputr
  @ 1. switch to svc mode
  msr cpsr_c, #0xd3
  @ 2. store user r0-r2 to svc stack for now, so we can use r0-r2 to transfer data
  stmfd sp!, {r0, r1, r2}
  @ 3. store magic number in r2
  mov r2, #0xab
  @ 4. store magic in hwi_request_flag location
  add r1, sp, #12
  ldmfd r1, {r0}
  str r2, [r0]
  @ 6. switch to irq mode, grab its lr_irq, which stores user's next pc, and spsr
  msr cpsr_c, #0x92
  mov r0, lr 
  mrs r1, spsr
  @ 7. switch to svc mode, move r0, r1's values to svc lr and spsr
  msr cpsr_c, #0xd3
  sub lr, r0, #4
  msr spsr, r1
  @@testing user's r0
  @mov r1, r2
  @mov r0, #1
  @bl bwputr
  @ 8. restore original user's r0, r1
  ldmfd sp!, {r0, r1, r2}
  @mov r1, r0
  @mov r0, #1
  @bl bwputr

kernelentry:
  @ 1. change to system state, sp lr are in user
  msr cpsr_c, #0xdf
  @ 2. store user's registers values onto user stack
  stmfd sp!, {r1-r12, r14}
  @ 3. move user's r0 value to r3
  mov r3, r0 
  @ 4. save active user task's sp in r1
  mov r1, sp
  @ 7. back to service state
  msr cpsr_c, #0xd3
  @ //TODO: move this to inline asm
  @ Calculate address of SWI instruction and load it into r0.
  ldr r0, [lr,#-4]
  @ Mask off top 8 bits of instruction to give SWI number.
  bic r0, r0,#0xFF000000
  @ stack user's next pc, it's stored in lr_svc by swi or hwikernelentry 
  stmfd r1!, {lr}
  @ 8. get spsr, which is user's cpsr set by swi or hwikernelentry 
  mrs r2, spsr
  @ 9. pop kernel registers from stack, jump to kernel's lr instruction
  @ pop off the hwi_flag off service stack
  add sp, sp, #4
  ldmfd sp!, {r4-r12, r14}
  mov sp, ip
  mov pc, lr
  @ r0: request_type,
  @ r1: user sp, 
  @ r2: user spsr, 
  @ r3: user's r0, used as default retval for hwi, ovewritten by swi
  @ user's arguments on user stack
