import asyncio
from websockets.asyncio.server import serve
from websockets.exceptions import ConnectionClosed, ConnectionClosedOK, ConnectionClosedError
from multiprocessing import Process, shared_memory, Lock, Event
import signal
import struct

MSG_HEADER_SIZE = 1 + struct.calcsize('H') # 3
MAX_MSG_LEN = 2**(8 * (MSG_HEADER_SIZE - 1)) - 1 # 65535
MAX_RX_MSG_SIZE = 100
MAX_TX_MSG_SIZE = 100
assert (MAX_RX_MSG_SIZE <= MAX_MSG_LEN)
assert (MAX_TX_MSG_SIZE <= MAX_MSG_LEN)
SEND_INTERVAL = 1/100

def send_text_msg(text_msg, lock_msg_tx, shm_msg_tx):
    bytes_msg = text_msg.encode('utf-8')
    msg_len = len(bytes_msg)
    if msg_len == 0 or msg_len > MAX_TX_MSG_SIZE:
        print("SIM: Length of text message to be sent is invalid. Message dropped!")
    else:
        with lock_msg_tx:
            shm_msg_tx.buf[0] = 2
            shm_msg_tx.buf[1:MSG_HEADER_SIZE] = struct.pack('H', msg_len)
            shm_msg_tx.buf[MSG_HEADER_SIZE:MSG_HEADER_SIZE + msg_len] = bytes_msg

class MPServer:
    def __init__(self, stop_event, shm_msg_tx_name, shm_msg_rx_name, lock_msg_tx, lock_msg_rx, 
                 server_ip = "10.0.0.124", 
                 server_port = 51347, 
                 use_localhost = True, 
                 auth_token = "jas73i46nuas6q02n3b5lsdjfoia"):
        self.stop_event = stop_event
        self.shm_msg_tx = shared_memory.SharedMemory(name=shm_msg_tx_name)
        self.shm_msg_tx_buf_size = len(self.shm_msg_tx.buf)
        self.shm_msg_rx = shared_memory.SharedMemory(name=shm_msg_rx_name)
        self.shm_msg_rx_buf_size = len(self.shm_msg_rx.buf)
        self.lock_msg_tx = lock_msg_tx
        self.lock_msg_rx = lock_msg_rx
        self.server_ip = server_ip
        self.server_port = server_port
        self.use_localhost = use_localhost
        self.auth_token = auth_token
        self.websocket = None

    async def _check_stop_event(self, server):
        try:
            while not self.stop_event.is_set():
                await asyncio.sleep(0.25)
            print("SERVER: Stop event received. Shutting down...")
        except (ConnectionClosed, ConnectionClosedOK, ConnectionClosedError):
            pass

    async def _server_receive(self):
        try:
            async for message in self.websocket:
                msg_len = len(message)
                if msg_len == 0 or msg_len > MAX_RX_MSG_SIZE:
                    print("Server: Length of message received is invalid. Message dropped!")
                else:
                    with self.lock_msg_rx:
                        buf = self.shm_msg_rx.buf
                        if isinstance(message, bytes):
                            buf[0] = 1 # Signals new bytes message received
                            buf[1:MSG_HEADER_SIZE] = struct.pack('H', msg_len)
                            buf[MSG_HEADER_SIZE:MSG_HEADER_SIZE + msg_len] = message
                        else: # message is str
                            print(f"SERVER: Text message received: {message}")
        except (ConnectionClosed, ConnectionClosedOK, ConnectionClosedError):
            pass

    async def _server_send(self):
        try:
            while not self.stop_event.is_set():
                out_msg = None
                buf = self.shm_msg_tx.buf
                if buf[0] != 0:
                    # First byte set indicates a new message is available for sending
                    with self.lock_msg_tx:
                        if buf[0] != 0: # Recheck after acquiring lock
                            msg_len = struct.unpack('H', buf[1:MSG_HEADER_SIZE])[0]
                            if msg_len == 0 or msg_len > MAX_TX_MSG_SIZE:
                                print("SERVER: Length of message to be sent is invalid. Message dropped!")
                            else:
                                out_msg = bytes(buf[MSG_HEADER_SIZE:MSG_HEADER_SIZE + msg_len]) # Copy message
                                is_bytes_msg = True if buf[0] == 1 else False
                            buf[0] = 0 # Clear message after copy
                    
                    if out_msg:
                        if is_bytes_msg:
                            asyncio.create_task(self.websocket.send(out_msg))
                            # await self.websocket.send(out_msg)
                        else:
                            await self.websocket.send(out_msg, text=True)
                            print(f"SERVER: Text message sent: {out_msg.decode('utf-8')}")
                
                if not out_msg:
                    await asyncio.sleep(0)
                
                # await asyncio.sleep(SEND_INTERVAL)
        except (ConnectionClosed, ConnectionClosedOK, ConnectionClosedError):
            pass

    async def _server_handler(self, websocket):
        try:
            print("SERVER: New connection established.")

            if self.websocket is not None:
                print("SERVER: Another connection already exists. Closing new connection.")
                await websocket.send("Connection occupied.")
                await websocket.close()
                return
            
            print("SERVER: Waiting for authentication token...")
            token = await websocket.recv()
            if token == self.auth_token:
                print("SERVER: Authorized client.\n")
                await websocket.send("Connection authorized.")
                self.websocket = websocket
            else:
                print("SERVER: Denied client. Closing connection.\n")
                await websocket.send("Connection denied.")
                await websocket.close()
                return

            task_receive = asyncio.create_task(self._server_receive())
            task_send = asyncio.create_task(self._server_send())
            await asyncio.gather(task_send, task_receive)
            
            print("SERVER: Connection closed.")
        except (ConnectionClosed, ConnectionClosedOK, ConnectionClosedError):
            pass
    
    async def _start_server(self):
        try:
            async with serve(self._server_handler, 
                            "localhost" if self.use_localhost is True else self.server_ip, 
                            self.server_port) as server:
                print("SERVER: Running.")

                stop_task = asyncio.create_task(self._check_stop_event(server))

                await stop_task

                if self.websocket is not None:
                    await self.websocket.close()
                server.close()
                await server.wait_closed()
                self.websocket = None
                self.shm_msg_rx.close()
                self.shm_msg_tx.close()
                print("SERVER: Shut down cleanly.")

                # try:
                #     await server.serve_forever()
                # except asyncio.CancelledError:
                #     pass
                # finally:
                #     stop_task.cancel()
                #     await server.wait_closed()
                #     self.websocket = None
                #     self.shm_msg_rx.close()
                #     self.shm_msg_tx.close()
                #     print("SERVER: Shut down cleanly.")
        except (ConnectionClosed, ConnectionClosedOK, ConnectionClosedError):
            pass

def start_mp_server(stop_event, shm_msg_tx_name, shm_msg_rx_name, lock_msg_tx, lock_msg_rx, 
                    server_ip = "10.0.0.124", 
                    server_port = 51347, 
                    use_localhost = False, 
                    auth_token = "jas73i46nuas6q02n3b5lsdjfoia"):
    # Ignore KeyboardInterrupt to support graceful shutdown in another process using Ctrl+C
    signal.signal(signal.SIGINT, signal.SIG_IGN)

    # Start server
    server = MPServer(stop_event, shm_msg_tx_name, shm_msg_rx_name, lock_msg_tx, lock_msg_rx, 
                      server_ip, server_port, use_localhost, auth_token)
    asyncio.run(server._start_server())

if __name__ == "__main__":
    try:
        # Start async server
        shm_msg_tx = shared_memory.SharedMemory(create=True, size=MSG_HEADER_SIZE + MAX_TX_MSG_SIZE)
        shm_msg_rx = shared_memory.SharedMemory(create=True, size=MSG_HEADER_SIZE + MAX_RX_MSG_SIZE)
        lock_msg_tx = Lock()
        lock_msg_rx = Lock()
        stop_event = Event()
        server_p_kwargs = {
            "stop_event" : stop_event,
            "shm_msg_tx_name" : shm_msg_tx.name,
            "shm_msg_rx_name" : shm_msg_rx.name,
            "lock_msg_tx" : lock_msg_tx,
            "lock_msg_rx" : lock_msg_rx,
            "server_ip" : "10.0.0.124", 
            "server_port" : 51347, 
            "use_localhost" : True, 
            "auth_token" : "jas73i46nuas6q02n3b5lsdjfoia"
        }
        server_p = Process(target=start_mp_server, kwargs=server_p_kwargs)
        server_p.start()

        while True:
            pass
    except KeyboardInterrupt:
        print("Interrupted — shutting down...")
    finally:
        # Ignore KeyboardInterrupt now to prevent shared memory leak in case someone gets impatient and spams Ctrl+C
        signal.signal(signal.SIGINT, signal.SIG_IGN)

        # Stop the server
        stop_event.set()
        server_p.join(timeout=3)

        # Terminate server if still alive
        if server_p.is_alive():
            print("Child didn't shut down cleanly, terminating...")
            server_p.terminate()
            server_p.join()
        
        # Clean up shared memory
        shm_msg_tx.close()
        shm_msg_tx.unlink()
        shm_msg_rx.close()
        shm_msg_rx.unlink()

        print("Cleanup done. Bye.")
