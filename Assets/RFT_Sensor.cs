using System;
using System.IO.Ports;
using System.Threading;
using System.Diagnostics;
using System.Collections.Generic;
//using MathNet.Numerics;
//using MathNet.Numerics.LinearAlgebra;
//using BurtSharp.Util.ClassExtensions;
//using UnityEngine;

namespace RFT
{
	public class RFT_Sensor
	{
		
		private EnhancedSerialPort serialPort;
		private Thread readThread;
		public List<ForceTorqueSample> FTSamples = new List<ForceTorqueSample>();
		public ForceTorqueSample latestFTSample;
		public float latestLoopTime = new float();
		byte[] readBuffer = new byte[4096];
		byte[] leftOverBytes;
		Stopwatch timestamper;

		private volatile bool lastCommandSuccess = false;

		public RFT_Sensor (string portName)
		{
			serialPort = new EnhancedSerialPort ();
			serialPort.PortName = "/dev/" + portName;//"/dev/ttyUSB0";
			serialPort.BaudRate = 921600;//460800  OR 921600
			serialPort.Parity = Parity.None;
			serialPort.DataBits = 8;
			serialPort.StopBits = StopBits.One;
			//serialPort.ReceivedBytesThreshold = 19;
			serialPort.DataReceived += new SerialDataReceivedEventHandler (DataReceivedHandler);


			//Set latency to 1
			ProcessStartInfo psi = new ProcessStartInfo();
			psi.FileName = "/bin/setserial";
			psi.UseShellExecute = false;
			psi.Arguments = "/dev/" + portName + " low_latency";
			Process p = Process.Start (psi);
			p.WaitForExit ();
			p.Close ();

			//check latency = 1
			psi = new ProcessStartInfo();
			psi.FileName = "/bin/cat";
			psi.UseShellExecute = false;
			psi.Arguments = "/sys/bus/usb-serial/devices/" + portName + "/latency_timer";
			psi.RedirectStandardOutput = true;
			p = Process.Start (psi);
			Console.Write ("Serial latency set to (should be 1): ");
			Console.WriteLine(p.StandardOutput.ReadToEnd());
			p.WaitForExit ();
			p.Close ();


			timestamper = new Stopwatch ();
			timestamper.Reset ();
			timestamper.Start ();

			leftOverBytes = new byte[0];

			try {
				serialPort.Open();
				Console.WriteLine ("Successfully opened serial port {0}",serialPort.PortName);
			} catch (Exception ex) {
				Console.WriteLine ("ERROR opening serial port");

			}

			sendSetBias (1);
			//sendStopFTData ();

//			readThread = new Thread (new ThreadStart(readWorker));
//			readThread.Priority = ThreadPriority.Highest;
//			readThread.Start ();
		}

//		~RFT_Sensor(){
//			serialPort.Close ();
//		}

		public void closePort()
		{
			serialPort.Close ();
		}

		public void sendReadModelName()
		{
			RFT_Packet packet = new RFT_Packet (PacketCommand.ReadModelName);
			sendPacket (ref packet);
		}

		public void sendStartFTData()
		{
			RFT_Packet packet = new RFT_Packet (PacketCommand.StartFTDataOutput);
			sendPacket (ref packet);
		}

		public void sendStopFTData()
		{
			RFT_Packet packet = new RFT_Packet (PacketCommand.StopFTDataOutput);
			sendPacket (ref packet);
		}
		public void sendSetBaudRate()
		{
			RFT_Packet packet = new RFT_Packet (PacketCommand.SetBaudRate,0x01);//hard coded as 921,600 for now
			sendPacket (ref packet);
		}
		public void sendSetOutputDataRate()
		{
			RFT_Packet packet = new RFT_Packet (PacketCommand.SetDataOutputRate, 8);//hard coded to 1000Hz for now
			sendPacket (ref packet);
		}
		public void sendReadOutputDataRate()
		{
			RFT_Packet packet = new RFT_Packet (PacketCommand.ReadDataOutputRate);
			sendPacket (ref packet);
		}
		public void sendReadBaudRate ()
		{
			RFT_Packet packet = new RFT_Packet (PacketCommand.ReadBaudRate);
			sendPacket (ref packet);
		}

		public void sendSetBias (byte setState) //setState = 0 => unset, 1 => set
		{
			RFT_Packet packet = new RFT_Packet (PacketCommand.SetBias, setState);
			sendPacket (ref packet);
		}

		public void sendSetFilter (byte setFilter)
		{
			RFT_Packet packet = new RFT_Packet (PacketCommand.SetFilter, setFilter);
			sendPacket (ref packet);
		}

		public void sendReadFilterSetting ()
		{
			RFT_Packet packet = new RFT_Packet (PacketCommand.ReadFilterSetting);
			sendPacket (ref packet);
		}

		private void sendPacket (ref RFT_Packet packet)
		{
			try{
				byte[] packetBuffer = packet.getPacketBuffer();
				serialPort.Write(packetBuffer,0,RFT_Packet.COMMAND_PACKET_LENGTH);
				Console.WriteLine("To {0}: {1}",serialPort.PortName,packetBuffer);
			} catch (Exception ex) {
				Console.WriteLine("ERROR writing to serial port");
			}

		}

		private void DataReceivedHandler(object sender, SerialDataReceivedEventArgs e)
		{

			Array.Copy (leftOverBytes, 0, readBuffer, 0, leftOverBytes.Length);
			int nNewBytes = serialPort.Read (readBuffer, leftOverBytes.Length, 128);
			int nBytes = nNewBytes + leftOverBytes.Length;
			int readIndex = 0;

			while (nBytes - readIndex >= RFT_Packet.RESPONSE_PACKET_LENGTH) {
				if (readBuffer [readIndex] == RFT_Packet.SOP) {
					byte[] potentialPacket = new byte[RFT_Packet.RESPONSE_PACKET_LENGTH];
					Array.Copy (readBuffer, readIndex, potentialPacket, 0, RFT_Packet.RESPONSE_PACKET_LENGTH);
					RFT_Packet receivedPacket = new RFT_Packet (potentialPacket);
					if(receivedPacket.packetCommand != 0){
						processReceivedPacket(ref receivedPacket,(float)timestamper.ElapsedTicks/(float)Stopwatch.Frequency);
					}
					readIndex += RFT_Packet.RESPONSE_PACKET_LENGTH;
				} else {
					readIndex++;
				}
			}

			leftOverBytes = new byte[nBytes-readIndex];
			Array.Copy (readBuffer, readIndex, leftOverBytes, 0, nBytes - readIndex);
		}
			
		private void processReceivedPacket(ref RFT_Packet receivedPacket, float timestamp)
		{
			//reorder for consistency
			switch (receivedPacket.packetCommand) 
			{
			case PacketCommand.SetDataOutputRate:
				if (receivedPacket.getSuccess ()) {
					lastCommandSuccess = true;
					Console.WriteLine ("Data output rate set successfully");
				} else {
					Console.WriteLine ("ERROR: Data output rate not set successfully");
				}
				break;
			case PacketCommand.ReadDataOutputRate:
				Console.WriteLine ("Data output rate is set to {0}", receivedPacket.getOutputRateData ());
				break;
			case PacketCommand.SetBaudRate:
				if (receivedPacket.getSuccess ()) {
					lastCommandSuccess = true;
					UnityEngine.Debug.Log ("Baud rate set successfully");
				} else {
					UnityEngine.Debug.Log ("ERROR: Baud rate not set successfully");
				}
				break;
			case PacketCommand.SetFilter:
				if (receivedPacket.getSuccess ()) {
					Console.WriteLine ("Filter set successfully");
				} else {
					Console.WriteLine ("ERROR: Filter not set successfully");
				}
				break;
			case PacketCommand.ReadFilterSetting:
				Console.WriteLine ("Filter is set to {0}", receivedPacket.getFilterStatus ());
				break;
			case PacketCommand.ReadFTDataOnce:
			case PacketCommand.StartFTDataOutput:
				float[] FTMeas = new float[6];
 					receivedPacket.getFTData (ref FTMeas);
//				ForceTorqueSample ftSamp = new ForceTorqueSample (FTMeas, timestampMillis);
//				FTSamples.Add (ftSamp);
				latestFTSample = new ForceTorqueSample(FTMeas, timestamp);
				break;
			default: //invalid command
				break;
			}
		
		}

		public string getForceDataAsString ()
		{
			string forceDataString = "";
			foreach (ForceTorqueSample samp in FTSamples) {
				forceDataString += samp.serialize ();
			}
			return forceDataString;
		}

		public void changeBaudRate()//right now set to max, parameters eventually
		{
			//need safeguards, i.e. "iswriting?"
			sendSetBaudRate();
			while (serialPort.BytesToWrite > 0) {
			}
			//serialPort.Close(); //need to write a nice way to handle this with other thread
			//serialPort.BaudRate = 921600;
//			while (!lastCommandSuccess) {
//			}
//			lastCommandSuccess = false;
//			try {
//				serialPort.Open();
//				Console.WriteLine ("Successfully opened serial port {0} at new baud rate",serialPort.PortName);
//			} catch (Exception ex) {
//				Console.WriteLine ("ERROR opening serial port at new baud rate");
//
//			}

		}
			
	}

	public class ForceTorqueSample {

		public float Fx, Fy, Fz, Tx, Ty, Tz;
		public float timestamp;

		public ForceTorqueSample(float[] ftMeas, float timestampMillis)
		{
			Fx = ftMeas [0];
			Fy = ftMeas [1];
			Fz = ftMeas [2];
			Tx = ftMeas [3];
			Ty = ftMeas [4];
			Tz = ftMeas [5];
			timestamp = timestampMillis;
		}

		public ForceTorqueSample(byte latestByte, float timestampMillis)
		{
			Fx = Fy = Fz = Tx = Ty = 0;
			Tz = (float)latestByte;
			timestamp = timestampMillis;
		}
		public string serialize()
		{
			return Fx.ToString () + ", " + Fy.ToString () + ", " + Fz.ToString () + ", " +
				Tx.ToString () + ", " + Ty.ToString () + ", " + Tz.ToString () + ", " + timestamp.ToString () + "\n";
		}

	}
}

