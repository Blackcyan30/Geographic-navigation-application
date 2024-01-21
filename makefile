build:
	rm -f application.exe
	g++ -std=c++20 -Wall application.cpp dist.cpp osm.cpp tinyxml2.cpp -o application.exe

run:
	./application.exe

buildtest:
	rm -f testing.exe
	g++ -std=c++20 -Wall testing.cpp -o testing.exe

runtest:
	./testing.exe

test:
	rm -f test.out
	g++ -std=c++20 test.cpp -o test.out -lgtest -lgtest_main

autoTest:
	# rm -f autoTest.out
	g++ -std=c++20 autoTest.cpp -o autoTest.out -lgtest -lgtest_main

testRun:
	./test.out

autoTestRun:
	./autoTest.out

clean:
	rm -f application.exe	

valgrind:
	valgrind --tool=memcheck --leak-check=yes ./application.exe
