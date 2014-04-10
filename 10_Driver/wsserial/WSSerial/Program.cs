using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

using System.IO.Ports;
using System.Net;
using Alchemy;
using Alchemy.Classes;

namespace WSSerial
{

    // TODO: Find a way to disconnect a socket from a UserContext without stopping the entire server.
    // TODO: Logging rather than exceptions.
    // TODO: Command line args for hostname and port.

    /// <summary>
    /// A user connection to a serial port.
    /// </summary>
    class WSConnection : IDisposable
    {
        private UserContext _Web { get; set; }
        private SerialPort _Serial = null;

        public WSConnection(UserContext pConnection, String Port, int BaudRate)
        {
            _Web = pConnection;
            _Serial = new SerialPort(Port, BaudRate, Parity.None, 8, StopBits.One);
            _Serial.DataReceived += Serial_DataReceived;
            _Serial.Open();

        }

        private void Serial_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            // Read the message.
            var sMessage = String.Empty;
            try
            {
                sMessage = _Serial.ReadExisting();
            }
            catch (Exception error)
            {
                Console.WriteLine("Error reading from serial. Dropped message.");
                return;
            }

            // Ship it out via the network.
            _Web.Send(sMessage);
        }

        public void Consume(UserContext context)
        {
            var message = context.DataFrame;
            if (message != null)
            {
                _Serial.Write(message.ToString());
            }
        }

        public void Dispose()
        {
            if (_Serial != null)
            {
                _Serial.Close();
                _Serial.Dispose();
                _Serial = null;
            }
        }
    }

    class Program
    {
        static Dictionary<UserContext, WSConnection> _Connections = new Dictionary<UserContext, WSConnection>();

        static void Main(string[] args)
        {
            //var host = args.Length > 2
            //var port = args.Length > 1 ? int.Parse(args[0]) : 81;
            var host = IPAddress.Any;
            var port = 81;

            // Set up the server.
            var Server = new WebSocketServer(81, IPAddress.Any)
            {
                OnReceive = OnReceive,
                OnConnected = OnConnected,
                OnDisconnect = OnDisconnect,
                TimeOut = new TimeSpan(0, 5, 0)
            };
            Server.Start();

            Console.WriteLine("WebSocket-Serial Bridge Started.");
            Console.WriteLine("\tvar ws = new WebSocket('ws://" + host + ":" + port + "/COM3/9600');");
            Console.WriteLine("\tws.send('hello world');");
            Console.ReadKey();
        }

        static void OnReceive(UserContext context)
        {
            // Defensive.
            if (!_Connections.ContainsKey(context))
                throw new Exception("Recieved data from previously closed connection.");

            // Consume the data (send it out via serial).
            _Connections[context].Consume(context);
        }

        static void OnConnected(UserContext context)
        {
            // Log.
            Console.WriteLine("Client Connection From : " + context.ClientAddress.ToString());
            Console.WriteLine("PATH: '" +context.RequestPath +"'");

            // Check for existing connection. Defensive.
            if (_Connections.ContainsKey(context))
            {
                var connection = _Connections[context];
                if (connection != null)
                    connection.Dispose();
            }

            // Trim leading '/' from path.
            var path = context.RequestPath.ToString();
            if (path.StartsWith("/"))
                path = path.Substring(1);

            // Check for correct args.
            var args = path.Split('/');
            if (args.Length != 2)
            {
                throw new Exception("Bad arugment number");
            }

            // Marshall args.
            var port = args[0].ToUpper();
            var baud = int.Parse(args[1]);

            // Check port exists.
            var lPorts = SerialPort.GetPortNames().Select((s) => { return s.ToUpper(); });
            if (!lPorts.Contains(port))
                throw new Exception(port + " does not exist");

            // Off we go!
            _Connections[context] = new WSConnection(context, port, baud);
        }

        static void OnDisconnect(UserContext context)
        {
            if (!_Connections.ContainsKey(context))
                throw new Exception("Recieved disconnect from previously closed connection.");

            _Connections[context].Dispose();
            _Connections.Remove(context);
            Console.WriteLine("Client Disconnection From : " + context.ClientAddress.ToString());
        }

    }
}
