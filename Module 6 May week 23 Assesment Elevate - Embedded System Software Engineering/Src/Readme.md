# Embedded System Software Engineering - C Programming Lab Repository

A comprehensive collection of C programming exercises, algorithms, and problem-solving implementations developed as part of the **Embedded System Software Engineering Assessment Program**.

This repository demonstrates practical understanding of:

- Core C programming concepts
- Embedded-system-oriented programming logic
- Algorithm implementation
- String and array manipulation
- Bitwise operations
- Structured programming methodology
- Problem-solving techniques

The project is designed to strengthen programming fundamentals required for embedded systems and low-level software development.

---

# Repository Overview

This repository contains categorized C programs covering:

## Basic Programming

- Even or Odd
- Prime Number Check
- Fibonacci Series
- Factorial Calculation
- Perfect Number
- Euclidean Algorithm

## String Processing

- Integer to String
- Palindrome String
- Pangram Check
- Reverse String
- Space Optimization
- Case Conversion

## Bit Manipulation

- Toggle Bits
- Extract Bits
- Swap Bits Between Numbers

## Array and Algorithm Programs

- Remove Duplicates
- Rotate Arrays
- Merge Sorted Arrays
- Find Pair with Given Sum
- Second Largest and Smallest

## Structure-Based Programs

- Complex Number Addition

---

# Technologies Used

- C Programming Language
- GCC Compiler
- Windows Batch Scripting
- Visual Studio Code

---

# Project Structure

```text
Embedded_SystemRoadMap/
│
├── Module 6 May week 23 Assesment Elevate - Embedded System Software Engineering/
│
└── Src/
    │
    ├── .vscode/
    │
    ├── run_all.bat
    ├── README.md
    │
    ├── 1_Even or Odd.c
    ├── 2Prime Number Check.c
    ├── 3_FactorialCalculation.c
    ├── 4_Fibonacci.c
    ├── 5_PerfectNumber .c
    ├── 6Euclidean Algorithm.c
    ├── 7FrequencyDigit.c
    ├── 8_Decimal to Any Base .c
    ├── 9_ArithmeticExpressionwithFunctions.c
    ├── 10_String of Digits to Integer.c
    │
    ├── 11_IntegertoString.c
    ├── 12_PalindromeString.c
    ├── 13_IterativeRS.c
    ├── 14_ PangramCheck.c
    ├── 15_Multiple SpaceswithSingleSpace.c
    ├── 16_ConvertUppercasetoLowercase.c
    │
    ├── 17_TogglenBitsromaPosition.c
    ├── 18_ExtractBitsPosition.c
    ├── 19_FirstNumber_SecondNumber.c
    ├── 20_Swap_n_Bits_Between2Numbers.c
    │
    ├── 21_RemoveDuplicateElements.c
    ├── 22SecondLargestandSecondSmallest.c
    ├── 23RotateArrayby-n-Positions.c
    ├── 24MergeTwoSortedArrays.c
    ├── 25FindPairswithGivenSum.c
    ├── 26PreservingOrder.c
    │
    └── 27ComplexNumberAddition.c
```

---

# Key Features

- Menu-driven execution system
- Automatic GCC compilation
- Automatic executable generation
- Organized modular program structure
- Beginner-friendly implementations
- Category-wise program grouping
- Practical algorithm implementations
- Batch automation for easy execution
- Embedded systems programming foundation

---

# GCC Compiler Installation

Before running the programs, install GCC Compiler.

## Recommended Compiler

WinLibs GCC:
https://winlibs.com/

## Alternative

MSYS2:
https://www.msys2.org/

---

# Verify GCC Installation

Open Command Prompt or VS Code terminal:

```bash
gcc --version
```

If installed correctly, GCC version information will appear.

---

# Running the Programs

## Method 1 - Using run_all.bat (Recommended)

### Step 1

Double-click:

```text
run_all.bat
```

### Step 2

Menu appears:

```text
=========================================
         C PROGRAM RUNNER
=========================================

BASIC PROGRAMS
-----------------------------------------
1.  Even or Odd
2.  Prime Number Check
3.  Factorial Calculation
4.  Fibonacci
5.  Perfect Number
6.  Euclidean Algorithm
7.  Frequency Digit
8.  Decimal to Any Base
9.  Arithmetic Expression with Functions
10. String of Digits to Integer

STRING PROGRAMS
-----------------------------------------
11. Integer to String
12. Palindrome String
13. Iterative Reverse String
14. Pangram Check
15. Multiple Spaces with Single Space
16. Convert Uppercase to Lowercase

BIT MANIPULATION PROGRAMS
-----------------------------------------
17. Toggle n Bits from a Position
18. Extract Bits from Position
19. First Number Second Number
20. Swap n Bits Between 2 Numbers

ARRAY PROGRAMS
-----------------------------------------
21. Remove Duplicate Elements
22. Second Largest and Smallest
23. Rotate Array by n Positions
24. Merge Two Sorted Arrays
25. Find Pairs with Given Sum
26. Preserving Order

STRUCTURE PROGRAMS
-----------------------------------------
27. Complex Number Addition
```

### Step 3

Enter program number.

Example:

```text
4
```

### Step 4

The script automatically:

- Compiles the selected C file
- Generates `program.exe`
- Executes the selected program

---

# Run Using VS Code Terminal

Open terminal in Visual Studio Code:

```cmd
.\run_all.bat
```

---

# Manual Compilation

Example:

```bash
gcc "4_Fibonacci.c" -o fibonacci.exe
```

Run:

```cmd
.\fibonacci.exe
```

---

# Compile and Run in One Command

```bash
gcc "4_Fibonacci.c" -o fib && .\fib.exe
```

---

# Programs Included

## Basic Programs

| No | Program |
|----|----------|
| 1 | Even or Odd |
| 2 | Prime Number Check |
| 3 | Factorial Calculation |
| 4 | Fibonacci |
| 5 | Perfect Number |
| 6 | Euclidean Algorithm |
| 7 | Frequency Digit |
| 8 | Decimal to Any Base |
| 9 | Arithmetic Expression with Functions |
| 10 | String of Digits to Integer |

---

## String Programs

| No | Program |
|----|----------|
| 11 | Integer to String |
| 12 | Palindrome String |
| 13 | Iterative Reverse String |
| 14 | Pangram Check |
| 15 | Multiple Spaces with Single Space |
| 16 | Convert Uppercase to Lowercase |

---

## Bit Manipulation Programs

| No | Program |
|----|----------|
| 17 | Toggle n Bits from a Position |
| 18 | Extract Bits from Position |
| 19 | First Number Second Number |
| 20 | Swap n Bits Between 2 Numbers |

---

## Array Programs

| No | Program |
|----|----------|
| 21 | Remove Duplicate Elements |
| 22 | Second Largest and Smallest |
| 23 | Rotate Array by n Positions |
| 24 | Merge Two Sorted Arrays |
| 25 | Find Pairs with Given Sum |
| 26 | Preserving Order |

---

## Structure Programs

| No | Program |
|----|----------|
| 27 | Complex Number Addition |

---

# Learning Outcomes

This repository helped strengthen understanding of:

- Procedural programming
- Structured problem solving
- Algorithm design
- String manipulation
- Bitwise operations
- Array processing
- Function modularization
- Input and output handling
- Compiler workflow
- Batch automation
- Embedded systems programming fundamentals

---

# Future Improvements

Planned future additions:

- Sorting algorithms
- Searching algorithms
- Linked List implementations
- Stack and Queue programs
- Dynamic Memory Allocation
- File Handling programs
- Embedded peripheral simulation examples
- Linux shell script support
- Cross-platform Makefile support

---

# Recommended VS Code Extensions

Recommended extensions for Visual Studio Code:

- C/C++
- Code Runner

VS Code Marketplace:
https://marketplace.visualstudio.com/vscode

---

# Educational Objective

The objective of this repository is to develop strong programming fundamentals required for:

- Embedded Systems Development
- Low-Level Software Engineering
- Firmware Development
- Algorithmic Problem Solving
- System Programming

The programs emphasize readable code, modular implementation, and practical problem-solving techniques.

---

# Author

Sreeraj Krishna K

Embedded Systems and Electronics Enthusiast

Interested in:

- Embedded Systems
- SDR (Software Defined Radio)
- Electronics Design
- Firmware Development
- Open Source Engineering

---

# License

This repository is intended for educational and learning purposes.