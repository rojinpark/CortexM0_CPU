gcc -c core.c
gcc -c inst.c
gcc -c memory.c
gcc -c thumb.c
gcc -c iss.c
gcc core.o inst.o memory.o thumb.o iss.o -o core -g
./core test.hex
