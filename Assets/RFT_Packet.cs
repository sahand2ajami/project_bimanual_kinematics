using System;

namespace RFT
{
	public enum PacketType {Command,
		Response};

	public enum PacketCommand {Invalid,
		ReadModelName,
		ReadSerialNumber,
		ReadFWVersion,
		SetCommID,
		ReadCommID,
		SetBaudRate,
		ReadBaudRate,
		SetFilter,
		ReadFilterSetting,
		ReadFTDataOnce,
		StartFTDataOutput,
		StopFTDataOutput,
		Reserved1,
		Reserved2,
		SetDataOutputRate,
		ReadDataOutputRate,
		SetBias,
		ReadOverloadCount};
	
	public class RFT_Packet
	{
		public const byte SOP = 0x55;
		public const byte EOP = 0xAA;

		public const float FORCE_DIVIDER = 50.0f;
		public const float TORQUE_DIVIDER = 2000.0f;

		public const int COMMAND_DATA_LENGTH = 8;
		public const int RESPONSE_DATA_LENGTH = 16;
		public const int COMMAND_PACKET_LENGTH = 3 + COMMAND_DATA_LENGTH;
		public const int RESPONSE_PACKET_LENGTH = 3 + RESPONSE_DATA_LENGTH;

		private byte[] packetData;

		public PacketType packetType;
		public PacketCommand packetCommand;

		public RFT_Packet ()
		{
		}

		public RFT_Packet (PacketCommand command)
		{
			packetData = new byte[COMMAND_DATA_LENGTH];
			packetType = PacketType.Command;
			packetCommand = command;

			switch (command) 
			{
			case PacketCommand.ReadModelName:
				packetData [0] = 0x01;
				break;
			case PacketCommand.ReadSerialNumber:
				packetData [0] = 0x02;
				break;
			case PacketCommand.StartFTDataOutput:
				packetData [0] = 0x0B;
				break;
			case PacketCommand.StopFTDataOutput:
				packetData [0] = 0x0C;
				break;
			case PacketCommand.ReadBaudRate:
				packetData [0] = 0x07;
				break;
			case PacketCommand.ReadDataOutputRate:
				packetData [0] = 0x10;
				break;
			case PacketCommand.ReadFilterSetting:
				packetData [0] = 0x09;
				break;
			default:
				Console.WriteLine ("ERROR, no behaviour for this command");
				break;
			}

		}

		public RFT_Packet (PacketCommand command, byte param1)
		{
			packetData = new byte[COMMAND_DATA_LENGTH];
			packetType = PacketType.Command;
			packetCommand = command;

			switch (command) 
			{
			case PacketCommand.SetBaudRate:
				packetData [0] = 0x06;
				packetData [1] = param1;
				break;
			case PacketCommand.SetDataOutputRate:
				packetData [0] = 0x0F;
				packetData [1] = param1;
				break;
			case PacketCommand.SetBias:
				packetData [0] = 0x11;
				packetData [1] = param1;
				break;
			case PacketCommand.SetFilter:
				packetData [0] = 0x08;
				packetData [1] = 0x01;
				packetData [2] = param1;
				break;
			default:
				Console.WriteLine ("ERROR, no behaviour for this command with 1 parameter");
				break;
			}

		}

		public RFT_Packet (byte[] data, byte checksum)
		{
			packetType = PacketType.Response;
			packetData = new byte[RESPONSE_DATA_LENGTH];
			packetData = data;
			if (calcChecksum () != checksum) { //if checksums don't match, packet is invalid
				packetData [0] = (byte)PacketCommand.Invalid;
			}
			packetCommand = (PacketCommand)packetData [0];
		}

		public RFT_Packet (byte[] receivedPacket)
		{
			packetType = PacketType.Response;
			packetData = new byte[RESPONSE_DATA_LENGTH];
			Array.Copy (receivedPacket, 1, packetData, 0, RESPONSE_DATA_LENGTH);
			if (calcChecksum () != receivedPacket[RESPONSE_DATA_LENGTH + 1]) { //if checksums don't match, packet is invalid
				packetData [0] = (byte)PacketCommand.Invalid;
			}
			packetCommand = (PacketCommand)packetData [0];
		}

		public byte[] getPacketBuffer()
		{
			//should be polymorphic...
			int packetBufferSize;
			if (packetType == PacketType.Command) {
				packetBufferSize = COMMAND_PACKET_LENGTH;
			} else {
				packetBufferSize = RESPONSE_PACKET_LENGTH;
			}
			
			byte[] packetBuffer = new byte[packetBufferSize];

			packetBuffer [0] = SOP;
			Array.Copy(packetData,0,packetBuffer,1,packetBufferSize - 3);
			packetBuffer [packetBufferSize - 2] = calcChecksum ();
			packetBuffer [packetBufferSize - 1] = EOP;

			return packetBuffer;

		}

		public byte calcChecksum()
		{
			byte checksum = 0;
			int dataLength;
			if (packetType == PacketType.Command) {
				dataLength = COMMAND_DATA_LENGTH;
			} else {
				dataLength = RESPONSE_DATA_LENGTH;
			}

			for (int i = 0; i < dataLength; i++) {
				checksum += packetData [i];			
			}

			return checksum;
		}

		public void getFTData(ref float[] FTMeas){

			ushort temp;

			for (int i = 0; i < 6; i++) {
				temp = (ushort)(packetData [2*(i+1)-1] * 256 + packetData [2*(i+1)]);
				if(i<3){
					FTMeas[i] = (float)((short)temp/FORCE_DIVIDER);
				}else{
					FTMeas[i] = (float)((short)temp/TORQUE_DIVIDER);
				}
			}
			 
		}

		public byte getFilterStatus() {
			if (packetData [1] == 1)
				return packetData [2];
			return packetData [1];
		}

		public bool getSuccess(){
			if(packetData[1] == 1){
				return true;
			}
			return false;
		}

		public byte getOutputRateData(){
			return packetData[1];
		}


	}
}

