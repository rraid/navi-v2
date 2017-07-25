all:
	g++ -c chili.cpp -o chili.o -fPIC -std=c++11 -O2 \
		`pkg-config --cflags opencv`
	g++ -shared -o libchili.so chili.o -std=c++11 -fPIC \
		`pkg-config --cflags opencv` `pkg-config --libs opencv` -lchilitags

clean:
	rm chili.o
	rm libchili.so
	rm *.pyc
