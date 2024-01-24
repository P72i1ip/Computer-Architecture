.globl __start

.rodata
    msg0: .string "This is HW1-1: T(n) = 5T(n/2) + 6n + 4, T(1) = 2\n"
    msg1: .string "Enter a number: "
    msg2: .string "The result is: "

.text


__start:
  # Prints msg0
    addi a0, x0, 4
    la a1, msg0
    ecall

  # Prints msg1
    addi a0, x0, 4
    la a1, msg1
    ecall

  # Reads an int
    addi a0, x0, 5
    ecall

################################################################################ 
  # Write your main function here. 
  # Input n is in a0. You should store the result T(n) into t0
  # HW1-1 T(n) = 5T(n/2) + 6n + 4, T(1) = 2, round down the result of division
  # ex. addi t0, a0, 1

Recursion:
  addi sp, sp, -8
  sw x1, 4(sp)
  sw x10, 0(sp)
  
  li x28, 2
  bge x10, x28, L1 #if n >= 2 goto L1

  #base case
  addi x5, x0, 2
  addi sp, sp, 8
  beq ra, x0, result
  ret

L1:
  div x10, x10, x28
  jal x1, Recursion
  addi x6, x5, 0

  lw x10, 0(sp) #pop original n from stack
  lw x1, 4(sp)  #pop ra from stack
  addi sp, sp, 8

  li x28, 5
  li x29, 6
  mul x10, x10, x29 #n = 6n
  mul x6, x6, x28 #T(n/2) = 5T(n/2)
  
  add x7, x10, x6 #temp = 5T(n/2)+6n
  addi x7, x7, 4  #temp += 4
  addi x5, x7, 0  #return T(n) = 5T(n/2)+6n+4
  beq ra, x0, result
  ret
################################################################################

result:
  # Prints msg2
    addi a0, x0, 4
    la a1, msg2
    ecall

  # Prints the result in t0
    addi a0, x0, 1
    add a1, x0, t0
    ecall
    
  # Ends the program with status code 0
    addi a0, x0, 10
    ecall
