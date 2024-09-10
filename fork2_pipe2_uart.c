#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <string.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>

#define BUFFER_SIZE 32

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

// Function to read from the serial port
void read_serial(int fd, int pipe_read) {
    char read_buffer[BUFFER_SIZE];
    int bytes_read;

    close(pipe_read); // Close unused write end

    while (1) {
        bytes_read = read(fd, read_buffer, sizeof(read_buffer) - 1);
        if (bytes_read > 0) {
            read_buffer[bytes_read] = '\0'; // Null-terminate the string
            printf("Received: %s", read_buffer); // Display the received data

            // Send received data to the pipe for logging
            write(pipe_read, read_buffer, bytes_read);

            if (strchr(read_buffer, '!')) {
                printf("Termination character '!' detected. Exiting read loop.\n");
                break; // Exit if '!' is detected
            }
        }
    }
}

// Function to write to the serial port
void write_serial(int fd, int pipe_write) {
    char input_buffer[BUFFER_SIZE];

    close(pipe_write); // Close unused read end

    while (1) {
        printf("Enter data to send (press 'q' to quit or 'test 1' to terminate parent): ");
        fgets(input_buffer, sizeof(input_buffer), stdin);
        input_buffer[strcspn(input_buffer, "\n")] = 0; // Remove newline character

        if (strcmp(input_buffer, "test 1") == 0) {
            write(fd, "!", 1); // Send '!' to terminate the reading process
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

    // Create pipes for inter-process communication
    int pipe_read[2]; // Pipe for reading data
    int pipe_write[2]; // Pipe for writing data
    pipe(pipe_read);
    pipe(pipe_write);

    pid_t pid_reader = fork();
    if (pid_reader < 0) {
        perror("Fork for reader failed");
        close(fd);
        return 1;
    }

    if (pid_reader == 0) {
        // Child process for reading from the serial port
        close(pipe_write[0]); // Close write end of the write pipe
        read_serial(fd, pipe_read[1]);
        exit(0);
    }

    pid_t pid_writer = fork();
    if (pid_writer < 0) {
        perror("Fork for writer failed");
        close(fd);
        return 1;
    }

    if (pid_writer == 0) {
        // Child process for writing to the serial port
        close(pipe_read[0]); // Close read end of the read pipe
        write_serial(fd, pipe_write[1]);
        exit(0);
    }

    // Parent process
    close(pipe_read[1]); // Close write end of the read pipe
    close(pipe_write[1]); // Close write end of the write pipe

    // Wait for both children to finish
    wait(NULL); // Wait for the reader to finish
    wait(NULL); // Wait for the writer to finish

    // Read from the read pipe
    char buffer[BUFFER_SIZE];
    while (read(pipe_read[0], buffer, sizeof(buffer)) > 0) {
        printf("Data from reader pipe: %s", buffer);
    }

    close(fd); // Close the serial port
    return 0;
}
