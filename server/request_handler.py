from fuzzy_system.moo_fuzzy_system import MooFuzzySystem
from fuzzy_system.simple_fuzzy_system import SimpleFuzzySystem
from misc.connection_helper import ConnectionHelper


class RequestHandler:
    def __init__(self):
        self.fuzzy_system = None
        self.method = None

    def handle_message(self, message):
        if self.method == "moo":
            u, w = self.fuzzy_system.run(message)
            return {
                "u": round(u, 4),
                "w": round(w, 4)
            }
        elif self.method == "simple":
            front, left, right, velocity = message["df"], message["dl"], message["dr"], message["velocity"]
            front = min(max(front, 0), 70)
            left = min(max(left, 0), 70)
            right = min(max(right, 0), 70)

            print(f'front : {front},left:{left},right:{right}')
            values = {
                "front": front,
                "left": left,
                "right": right,
                "velocity": velocity
            }
            velocity, angle = self.fuzzy_system.run(values)
            # angle = radians(angle)
            return {
                "velocity": velocity,
                "angle": angle
            }

    def start(self, client_socket):
        try:
            message = ConnectionHelper.receive_json(client_socket)
            print(f"received message: {message}")
            method = message["method"].lower()
            if method == "moo":
                self.fuzzy_system = MooFuzzySystem(use_lex=False)
            elif method == "simple":
                self.fuzzy_system = SimpleFuzzySystem()
            self.method = method
            ConnectionHelper.send_json(client_socket, {
                "method": method
            })
            while True:
                message = ConnectionHelper.receive_json(client_socket)
                if message is None:
                    break
                print(f"received message: {message}")
                result = self.handle_message(message)
                ConnectionHelper.send_json(client_socket, result)
                print(f"result:{result}")
        finally:
            print("socket closed")
            client_socket.close()
