all:
	g++ -o tune_pid pid_tuning.cpp \
	 -I/usr/local/include/plplot -lplplotcxxd -lplplotd \
	 -std=c++11 -pthread -O3

