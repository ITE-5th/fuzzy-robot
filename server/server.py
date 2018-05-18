import os
import socket
import threading

try:
    from server.request_handler import RequestHandler
except:
    from request_handler import RequestHandler


class LocalServer:
    def __init__(self, host=socket.gethostname(), port=8888, use_lex=True):
        self.host = host
        self.port = port
        self.use_lex = use_lex
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.bind((host, port))
        self.socket.listen(5)
        self.client_socket, self.address = None, None

    def handle_client_connection(self, client_socket):
        handler = RequestHandler(self.use_lex)
        handler.start(client_socket)

    def start(self):
        print('server started at {}:{}'.format(self.host, str(self.port)))
        while True:
            client_socket, address = self.socket.accept()
            print('Accepted connection from {}:{}'.format(address[0], address[1]))
            client_handler = threading.Thread(
                target=self.handle_client_connection,
                args=(client_socket,)
            )
            client_handler.start()

    def close(self):
        self.socket.close()


if __name__ == '__main__':
    os.system('ps -fA | grep python | tail -n1 | awk \'{ print $3 }\'|xargs kill')
    # server = LocalServer(port=9999)
    server = LocalServer(host="192.168.1.7", port=8888)
    try:
        server.start()
    finally:
        server.close()
