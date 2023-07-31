#include <iostream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>

int main(int argc, char const *argv[]) {
    int sock = 0, port = 9709;
    struct sockaddr_in serv_addr;

	if (argc < 3) {
		std::cerr << "usage: ./client <num_trials> <ip> [port]" << std::endl;
		return -1;
	}
    const char* num_trials = argv[1];
    const char* ip = argv[2];

	if (argc == 4)
		port = atoi(argv[3]);

    // Create socket file descriptor
    if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        std::cerr << "Socket creation error" << std::endl;
        return -1;
    }

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(port);

    // Convert IPv4 and IPv6 addresses from text to binary form
    if(inet_pton(AF_INET, ip, &serv_addr.sin_addr)<=0) {
        std::cerr << "Invalid address/ Address not supported" << std::endl;
        return -1;
    }

    // Connect to the server
    if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
        std::cerr << "Connection Failed" << std::endl;
        return -1;
    }

    // Send message to the server
    send(sock, num_trials, strlen(num_trials), 0);

    return 0;
}
