#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <string.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>

// Function to set up the serial port
void setup_serial(int fd) {
    struct termios SerialPortSettings;
    tcgetattr(fd, &SerialPortSettings);
    cfsetispeed(&SerialPortSettings, B115200);
    cfsetospeed(&SerialPortSettings, B115200);
    SerialPortSettings.c_cflag &= ~PARENB;   // No parity
    SerialPortSettings.c_cflag &= ~CSTOPB;   // 1 stop bit
    SerialPortSettings.c_cflag &= ~CSIZE;     // Clear size bits
    SerialPortSettings.c_cflag |= CS8;        // 8 data bits
    SerialPortSettings.c_cflag |= CREAD | CLOCAL; // Enable receiver and ignore modem control lines

    SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY); // Disable software flow control
    SerialPortSettings.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // Non-canonical mode
    SerialPortSettings.c_oflag &= ~OPOST; // Raw output

    SerialPortSettings.c_cc[VMIN] = 1; // Minimum number of characters to read
    SerialPortSettings.c_cc[VTIME] = 0; // No timeout

    tcsetattr(fd, TCSANOW, &SerialPortSettings);
}

// Function to read from the serial port in the parent process
void read_serial(int fd) {
    char read_buffer[32];
    int bytes_read;

    while (1) {
        bytes_read = read(fd, read_buffer, sizeof(read_buffer) - 1);
        if (bytes_read > 0) {
            read_buffer[bytes_read] = '\0'; // Null-terminate the string
            printf("Received: %s", read_buffer); // Display the received data
            if (strchr(read_buffer, '!')) {
                printf("Termination character '!' detected. Exiting read loop.\n");
                break; // Exit if '!' is detected
            }
        }
    }
}

// Function to write to the serial port in the child process
void write_serial(int fd) {
    char input_buffer[32];

    while (1) {
        printf("Enter data to send (press 'q' to quit or 'test 1' to terminate parent): ");
        fgets(input_buffer, sizeof(input_buffer), stdin);
        input_buffer[strcspn(input_buffer, "\n")] = 0; // Remove newline character

        if (strcmp(input_buffer, "test 1") == 0) {
            write(fd, "!", 1); // Send '!' to the parent
            printf("Sent termination character '!' to parent.\n");
        } else {
            write(fd, input_buffer, strlen(input_buffer)); // Send the entered data
        }

        if (strcmp(input_buffer, "q") == 0) {
            printf("Termination character 'q' detected. Exiting write loop.\n");
            break; // Exit if 'q' is detected
        }
    }
}

int main() {
    int fd = open("/dev/ttyS1", O_RDWR | O_NOCTTY);
    if (fd == -1) {
        perror("Failed to open serial port");
        return 1;
    }

    // Set up the serial port
    setup_serial(fd);

    pid_t pid = fork();
    if (pid < 0) {
        perror("Fork failed");
        close(fd);
        return 1;
    }

    if (pid == 0) {
        // Child process: Write to the serial port
        write_serial(fd);
        exit(0);
    } else {
        // Parent process: Read from the serial port
        read_serial(fd);
        wait(NULL); // Wait for the child process to finish
    }

    close(fd); // Close the serial port
    return 0;
}
