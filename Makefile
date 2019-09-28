all:
	#clang -emit-llvm -g -O0 -Xclang -disable-O0-optnone iso7816_platform.c smartcard_iso7816.c smartcard.c -o main.bc
	$(CC) smartcard_iso7816.c -o main
clean:
	@rm -rf *.o
