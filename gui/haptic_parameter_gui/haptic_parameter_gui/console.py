import json


class ConsoleTransport:
    def publish(self, parameters):
        print(json.dumps(parameters, indent=2, sort_keys=True))
