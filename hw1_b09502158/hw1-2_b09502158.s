.globl __start

.rodata
    msg0: .string "This is HW1-2: \n"
    msg1: .string "Enter shift: "
    msg2: .string "Plaintext: "
    msg3: .string "Ciphertext: "
.text

################################################################################
  # print_char function
  # Usage: 
  #     1. Store the beginning address in x20
  #     2. Use "j print_char"
  #     The function will print the string stored from x20 
  #     When finish, the whole program with return value 0

print_char:
    addi a0, x0, 4
    la a1, msg3
    ecall
  
    add a1,x0,x20
    ecall

  # Ends the program with status code 0
    addi a0,x0,10
    ecall
    
################################################################################

__start:
  # Prints msg
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
    add a6, a0, x0
    
  # Prints msg2
    addi a0, x0, 4
    la a1, msg2
    ecall
    
    addi a0,x0,8
    li a1, 0x10150
    addi a2,x0,2047
    ecall
  # Load address of the input string into a0
    add a0,x0,a1


################################################################################ 
  # Write your main function here. 
  # a0 stores the begining Plaintext
  # x16(a6) stores the shift
  # Do store 66048(0x10200) into x20 
  # ex. j print_char

  li x20, 0x10200
  mv t6, x20  #begin of the encrypted output

  li t4, 0  #number of the space

  Shift_check:  #deal with negative shift
    bge a6, x0, Encrypt_loop
    addi a6, a6, 26

  Encrypt_loop:
    lbu t1, 0(a0) #t1 = current character

    li t2, 10 #(t2 = '\n')
    beq t1, t2, End #plaintext ends with '\n'

    li t2, 32 #t2 = space
    beq t1, t2, space

    addi t1, t1, -97  #move a~z to 0~25
    add t1, t1, a6  #plus the shift

    li t2, 26 #t2 = 26
    remu t1, t1, t2 #take the remainder
    addi t1, t1, 97 #shift back to original ascii
    
    sb t1, 0(t6)
    addi t6, t6, 1
    addi a0, a0, 1  #next character
    j Encrypt_loop


  space:
    addi t5, t4, 48 #ascii for the number of space
    mv t1, t5
    sb t1, 0(t6)
    addi t6, t6, 1
    addi a0, a0, 1  #next character
    addi t4, t4, 1  #increase the number of space by 1
    j Encrypt_loop

  End:
    j print_char
  
################################################################################

