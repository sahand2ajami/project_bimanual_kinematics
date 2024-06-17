using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Text;
using System.IO.Ports;

namespace TriggerEMG
{
    public class CreateArdunioConnection
	 {
        SerialPort port;

		public CreateArdunioConnection (string portName)
		{
            if (port == null)
            {
                port = new SerialPort(portName, 9600, Parity.None, 8, StopBits.One);
                port.Open();
                //port.Write("#STAR\n");
                //Console.WriteLine("Port Opend?  " + port.IsOpen);
            }
		}

        public void writeToArduino(string massage)
        {
            if (port != null && port.IsOpen)
            {
                port.Write(massage);
				//Console.WriteLine("massege succesffuly sent:  " + massage);
            }
        }


        public void disconnectFromArduino()
        {
            if (port != null && port.IsOpen)
            {
                port.Close();
                //Console.WriteLine("Port succesffuly closed.");
            }
        }

        
    }
}
