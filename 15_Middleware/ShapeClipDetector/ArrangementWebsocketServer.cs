using Alchemy;
using Alchemy.Classes;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Net;
using System.Text;
using System.Threading.Tasks;

class ArrangementWebsocketServer : WebSocketServer
{
    private const int FRAMERATE = 10;

    private List<UserContext> clients = new List<UserContext>();
    private byte[] lastFrame;
    private readonly object frameLock = new object();

    public ArrangementWebsocketServer(int port)
        : base(port, IPAddress.Any)
    {
        OnConnect += (UserContext client) =>
        {
            clients.Add(client);
            Console.WriteLine("-> {0} requesting {1}", client.ClientAddress, client.RequestPath);
        };
        OnReceive += (UserContext client) =>
        {
            if (client.RequestPath.EndsWith("sdsl"))
            {
                foreach (UserContext ctx in clients)
                    if (ctx.RequestPath.EndsWith("shader"))
                    {
                        List<ArraySegment<byte>> arrays = ctx.DataFrame.AsRaw();
                        byte[] rv = new byte[arrays.Sum(array => array.Array.Length)];
                        int offset = 0;
                        foreach (byte[] current in arrays.Select(array => array.Array))
                        {
                            Buffer.BlockCopy(current, 0, rv, offset, current.Length);
                            offset += current.Length;
                        }
                        Console.WriteLine(rv.Length);

                        //ctx.Send(client.DataFrame, true);
                    }
            }
        };
        OnDisconnect += (UserContext client) => clients.Remove(client);
        TimeOut = new TimeSpan(8, 0, 0);
    }


    public void BroadcastArrangement(ShapeClip[] clips)
    {
        lock (frameLock)
        {
            byte[] frame = BuildSDSLFrame(clips);
            foreach (UserContext ctx in clients)
            {
                try
                {
                    if (ctx.RequestPath.EndsWith("sdsl")) ctx.Send(frame, true, false);
                }
                catch (NullReferenceException e)
                {
                    Console.WriteLine(" Alchemy hickup: NullReferenceException");
                }
            }

            frame = BuildDriverFrame(clips);
            foreach (UserContext ctx in clients)
            {
                try
                {
                    if (ctx.RequestPath.EndsWith("driver")) 
                        ctx.Send(frame, true, false);
                }
                catch (NullReferenceException e)
                {
                    Console.WriteLine(" Alchemy hickup: NullReferenceException");
                }
            }

            lastFrame = frame;
        }
    }

    private byte[] BuildSDSLFrame(ShapeClip[] clips)
    {
        MemoryStream stream = new MemoryStream();
        stream.WriteByte((byte)0x02 + 128);

        // (3 positionComponents + 3 normalComponents) * (4 floatBytes)
        var payload_length = clips.Length * (3 + 3) * 4;
        if (payload_length > UInt16.MaxValue)
        {
            stream.WriteByte(127);
            var lengthBytes = ToBigEndianBytes<ulong>(payload_length);
            stream.Write(lengthBytes, 0, lengthBytes.Length);
        }
        else if (payload_length > 125)
        {
            stream.WriteByte(126);
            var lengthBytes = ToBigEndianBytes<ushort>(payload_length);
            stream.Write(lengthBytes, 0, lengthBytes.Length);
        }
        else
        {
            stream.WriteByte((byte)payload_length);
        }

        for (int i = 0; i < clips.Length; i++)
        {
            ShapeClip clip = clips[i];

            // position
            foreach (float comp in clip.Position.ToArray()) stream.Write(BitConverter.GetBytes(comp), 0, 4);
            stream.Write(BitConverter.GetBytes(0.0f), 0, 4);

            // normal
            stream.Write(BitConverter.GetBytes(0.0f), 0, 4);
            stream.Write(BitConverter.GetBytes(0.0f), 0, 4);
            stream.Write(BitConverter.GetBytes(1.0f), 0, 4);
        }

        return stream.ToArray();
    }

    private byte[] BuildDriverFrame(ShapeClip[] clips)
    {
        MemoryStream stream = new MemoryStream();
        stream.WriteByte((byte)0x02 + 128);

        // (2 positionComponents + 1 rotationComponents) * (4 floatBytes)
        var payload_length = clips.Length * (2 + 1) * 4;
        if (payload_length > UInt16.MaxValue)
        {
            stream.WriteByte(127);
            var lengthBytes = ToBigEndianBytes<ulong>(payload_length);
            stream.Write(lengthBytes, 0, lengthBytes.Length);
        }
        else if (payload_length > 125)
        {
            stream.WriteByte(126);
            var lengthBytes = ToBigEndianBytes<ushort>(payload_length);
            stream.Write(lengthBytes, 0, lengthBytes.Length);
        }
        else
        {
            stream.WriteByte((byte)payload_length);
        }

        for (int i = 0; i < clips.Length; i++)
        {
            ShapeClip clip = clips[i];

            // position
            stream.Write(BitConverter.GetBytes(clip.Position.X), 0, 4);
            stream.Write(BitConverter.GetBytes(clip.Position.Y), 0, 4);
            
            // rotation
            stream.Write(BitConverter.GetBytes((float)clip.Angle), 0, 4);
        }

        return stream.ToArray();
    }

    public static byte[] ToBigEndianBytes<T>(int source)
    {
        byte[] bytes;

        var type = typeof(T);
        if (type == typeof(ushort))
            bytes = BitConverter.GetBytes((ushort)source);
        else if (type == typeof(ulong))
            bytes = BitConverter.GetBytes((ulong)source);
        else if (type == typeof(int))
            bytes = BitConverter.GetBytes(source);
        else
            throw new InvalidCastException("Cannot be cast to T");

        if (BitConverter.IsLittleEndian)
            Array.Reverse(bytes);
        return bytes;
    }

}

