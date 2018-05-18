from fuzzy_controller.fuzzy_system import FuzzySystem
from misc.connection_helper import ConnectionHelper


class RequestHandler:
    def __init__(self, use_lex=False):
        self.fuzzy_system = FuzzySystem()
        self.use_lex = use_lex

    def handle_message(self, message):
        dl, df, dr, a, p, ed = message["dl"], message["df"], message["dr"], message["alpha"], message["p"], message[
            "ed"]

        u, w = self.fuzzy_system.run(dl, df, dr, a, p, ed, use_lex=self.use_lex)
        return {
            "u": u,
            "w": w
        }

    def start(self, client_socket):
        try:
            while True:
                message = ConnectionHelper.receive_json(client_socket)
                if message is None:
                    break
                print(f"received message : {message}")
                result = self.handle_message(message)
                ConnectionHelper.send_json(client_socket, result)
                print("result:")
                print(result)
        finally:
            print("socket closed")
            client_socket.close()
