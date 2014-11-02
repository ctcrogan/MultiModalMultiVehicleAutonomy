#using <System.dll>

using namespace System;
using namespace System::Runtime::InteropServices;
using namespace System::IO::Ports;
using namespace System::Threading;


public ref class PortChat
{
private:
    static SerialPort^ _serialPort;
	static String^ GPScoords;
	static bool _continue;

public:
	static char* getGPScoords()
	{
		// Stop Reading in new Data
		_continue = false;

		// Marshal the managed string to unmanaged memory. 
		char* stringPointer = (char*) Marshal::StringToHGlobalAnsi(GPScoords).ToPointer();
		return stringPointer;
	}

    static void startChat()
    {
        //String^ message;
        StringComparer^ stringComparer = StringComparer::OrdinalIgnoreCase;
        Thread^ readThread = gcnew Thread(gcnew ThreadStart(PortChat::Read));

        // Create a new SerialPort object with default settings.
        _serialPort = gcnew SerialPort("COM11", 115200, (Parity)0, 8, (StopBits)1);

        // Set the read/write timeouts
        _serialPort->ReadTimeout = 500;
        _serialPort->WriteTimeout = 500;
		_serialPort->NewLine = "\r";

        _serialPort->Open();
        _continue = true;
        readThread->Start();

        /*while (_continue)
        {
            try
            {
				message = Console::ReadLine();

				if (stringComparer->Equals("quit", message))
				{
					_continue = false;
				}
				else
				{
					_serialPort->Write("Hello");
				}
            }
            catch (TimeoutException ^) { }
        }*/

        readThread->Join();
        _serialPort->Close();
    }

    static void Read()
    {
        while (_continue)
        {
            try
            {
                String^ message = _serialPort->ReadLine();
			
				if(message[1] == 35 && message[2] == 53)
				{
					GPScoords = message;
					Console::WriteLine(message);
				}
            }
            catch (TimeoutException ^) { }
        }
    }

};