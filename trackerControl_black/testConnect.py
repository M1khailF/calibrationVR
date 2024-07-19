import motorcortex
parameter_tree = motorcortex.ParameterTree()

try:
    req, sub = motorcortex.connect("wss://" + "192.168.2.100" + ":5568:5567",
                                                motorcortex.MessageTypes(), parameter_tree,
                                                certificate="mcx.cert.pem", timeout_ms=1000,
                                                login="admin", password="vectioneer")
    tree = parameter_tree.getParameterTree()
    print(f"Parameters: {tree}")

except RuntimeError as err:
    print(err)