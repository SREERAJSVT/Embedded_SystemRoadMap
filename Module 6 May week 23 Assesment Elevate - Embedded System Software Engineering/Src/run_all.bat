@echo off
color 0A
title C PROGRAM RUNNER

:menu
cls
echo =========================================
echo         C PROGRAM RUNNER
echo =========================================
echo.
echo BASIC PROGRAMS
echo -----------------------------------------
echo 1.  Even or Odd
echo 2.  Prime Number Check
echo 3.  Factorial Calculation
echo 4.  Fibonacci
echo 5.  Perfect Number
echo 6.  Euclidean Algorithm
echo 7.  Frequency Digit
echo 8.  Decimal to Any Base
echo 9.  Arithmetic Expression with Functions
echo 10. String of Digits to Integer
echo.
echo STRING PROGRAMS
echo -----------------------------------------
echo 11. Integer to String
echo 12. Palindrome String
echo 13. Iterative Reverse String
echo 14. Pangram Check
echo 15. Multiple Spaces with Single Space
echo 16. Convert Uppercase to Lowercase
echo.
echo BIT MANIPULATION PROGRAMS
echo -----------------------------------------
echo 17. Toggle n Bits from a Position
echo 18. Extract Bits from Position
echo 19. First Number Second Number
echo 20. Swap n Bits Between 2 Numbers
echo.
echo ARRAY PROGRAMS
echo -----------------------------------------
echo 21. Remove Duplicate Elements
echo 22. Second Largest and Smallest
echo 23. Rotate Array by n Positions
echo 24. Merge Two Sorted Arrays
echo 25. Find Pairs with Given Sum
echo 26. Preserving Order
echo.
echo STRUCTURE PROGRAMS
echo -----------------------------------------
echo 27. Complex Number Addition
echo.
echo 0. Exit
echo.

set /p choice=Enter your choice: 

if "%choice%"=="1" set file=1_Even or Odd.c
if "%choice%"=="2" set file=2Prime Number Check.c
if "%choice%"=="3" set file=3_FactorialCalculation.c
if "%choice%"=="4" set file=4_Fibonacci.c
if "%choice%"=="5" set file=5_PerfectNumber .c
if "%choice%"=="6" set file=6Euclidean Algorithm.c
if "%choice%"=="7" set file=7FrequencyDigit.c
if "%choice%"=="8" set file=8_Decimal to Any Base .c
if "%choice%"=="9" set file=9_ArithmeticExpressionwithFunctions.c
if "%choice%"=="10" set file=10_String of Digits to Integer.c
if "%choice%"=="11" set file=11_IntegertoString.c
if "%choice%"=="12" set file=12_PalindromeString.c
if "%choice%"=="13" set file=13_IterativeRS.c
if "%choice%"=="14" set file=14_ PangramCheck.c
if "%choice%"=="15" set file=15_Multiple SpaceswithSingleSpace.c
if "%choice%"=="16" set file=16_ConvertUppercasetoLowercase.c
if "%choice%"=="17" set file=17_TogglenBitsromaPosition.c
if "%choice%"=="18" set file=18_ExtractBitsPosition.c
if "%choice%"=="19" set file=19_FirstNumber_SecondNumber.c
if "%choice%"=="20" set file=20_Swap_n_Bits_Between2Numbers.c
if "%choice%"=="21" set file=21_RemoveDuplicateElements.c
if "%choice%"=="22" set file=22SecondLargestandSecondSmallest.c
if "%choice%"=="23" set file=23RotateArrayby-n-Positions.c
if "%choice%"=="24" set file=24MergeTwoSortedArrays.c
if "%choice%"=="25" set file=25FindPairswithGivenSum.c
if "%choice%"=="26" set file=26PreservingOrder.c
if "%choice%"=="27" set file=27ComplexNumberAddition.c

if "%choice%"=="0" exit

if not defined file (
    echo.
    echo Invalid Choice!
    pause
    goto menu
)

cls
echo =========================================
echo Compiling %file%
echo =========================================
echo.

gcc "%file%" -o program.exe

if %errorlevel% neq 0 (
    echo.
    echo Compilation Failed!
    pause
    goto menu
)

cls
echo =========================================
echo Running %file%
echo =========================================
echo.

program.exe

echo.
echo =========================================
pause
goto menu