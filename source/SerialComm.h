#ifndef SERIALCOMM_H
#define SERIALCOMM_H
#include <string>
#include "AsyncSerial.h"






class EcsSerialCommunicator : public boost::noncopyable
{
public:
    EcsSerialCommunicator(std::string serial_port_name = "COM15", unsigned long baud_rate_sel = 115200)
        :serial_comm(serial_port_name, baud_rate_sel)
    {
        splitter_old.assign({ -1, -1 });
        if (!serial_comm.isOpen())
            std::cout << "Warning: serial port " << serial_port_name << "cannot be opened." << std::endl
            << "Please provide another serial port with SetSerial ASAP." << std::endl;
        else
        {
            serial_comm.writeString("IK\n");
        }

        serial_comm.setCallback(boost::bind(&EcsSerialCommunicator::onSerialReceive,this,_1,_2));
        //boost::function<void(const char*, int)>

    }

    ~EcsSerialCommunicator() {}

    void SetSerial(std::string serial_port_name = "COM15", unsigned long baud_rate_sel = 115200)
    {
        serial_comm.open(serial_port_name, baud_rate_sel);
        if (!serial_comm.isOpen())
            std::cout << "Warning: serial port " << serial_port_name << "cannot be opened." << std::endl
            << "Please provide another serial port with SetSerial ASAP." << std::endl;
    }

    enum class message_output_t
    {
        MOTOR_HALT,
        MOTOR_STOP,
        RETRIEVE_POSX,
        RETRIEVE_POSY,
        RETRIEVE_POS
    };

    void SendCommand(message_output_t command_to_be_sent)
    {
        std::stringstream to_be_sent;
        switch (command_to_be_sent)
        {
            case message_output_t::MOTOR_HALT: to_be_sent << "HH"; break;
            case message_output_t::MOTOR_STOP: to_be_sent << "SS"; break;
            case message_output_t::RETRIEVE_POSX: to_be_sent << "QX"; break;
            case message_output_t::RETRIEVE_POSY: to_be_sent << "QY"; break;
            case message_output_t::RETRIEVE_POS: to_be_sent << "QP"; break;
            default: break;
        }

        writeString(to_be_sent);
    }


    void UpdateMotorPosition(std::array<ElectrostaticCoronaSeparator::splitter_t, 3> splitter)
    {
        std::stringstream to_be_sent_stream;
        to_be_sent_stream << std::setprecision(2);

        std::string splitter_prefix;
        for (auto split_sel = 0; split_sel < splitter.size(); ++split_sel)
        {
            switch (split_sel)
            {
                case 0:
                    splitter_prefix = "A"; break;
                case 1:
                    splitter_prefix = "B"; break;
                case 2:
                    splitter_prefix = "C"; break;
                default:
                    break;
            }

            // update distance from the drum
            if (abs(splitter[split_sel].pos_x - splitter_old[split_sel].pos_x) > 1e-2)
            {
                splitter_old[split_sel].pos_x = splitter[split_sel].pos_x;
                to_be_sent_stream << "X" << splitter_prefix;
                to_be_sent_stream << splitter[split_sel].pos_x;
                writeString(to_be_sent_stream);
                to_be_sent_stream.str(std::string());
            }

            // update height of the splitter
            if (abs(splitter[split_sel].pos_y - splitter_old[split_sel].pos_y) > 1e-2)
            {
                splitter_old[split_sel].pos_y = splitter[split_sel].pos_y;
                to_be_sent_stream << "Y" << splitter_prefix;
                to_be_sent_stream << splitter[split_sel].pos_y;
                writeString(to_be_sent_stream);
                to_be_sent_stream.str(std::string());
            }

        }

    }


private:
    void onSerialReceive(const char* data, int data_size) {

        while (data_size>=0 && data[data_size-1] == '\n' || data[data_size-1] == '\r')
            --data_size;

        if (data_size<3)
        {
            std::cout << "Received misformatted packet" << std::endl;
            return;
        }

        std::string rec_str(data, data_size);

        std::stringstream str_str;
        try
        {
            switch (data[0])
            {
                case 'R':
                    str_str << "Motor " << data[2] << " in position " << data[1] << ": "<< rec_str.substr(3, rec_str.size() - 2) << std::endl;
                    break;
                case 'A':
                    if (data[1] == 'C' && data[2] == 'K')
                    {
                        if (ack_to_receive > 0)
                            ack_to_receive--;
                        str_str << "ACK received" << std::endl;
                    }
                    break;
                case 'I':
                    str_str << "INFO from Splitter Controller : ";
                    switch (data[1])
                    {
                        case 'H':
                            str_str << "Motor " << data[2] << " halted." << std::endl; break;
                        case 'S':
                            str_str << "Motor " << data[2] << " stopped." << std::endl; break;
                        case 'R':
                            str_str << "Motor " << data[2] << " reset." << std::endl; break;
                        case 'O':
                            str_str << "Motor " << data[2] << " is asking too much current." << std::endl; break;
                        default:
                            break;
                    }
                    break;
                case 'E':
                    str_str << "INFO from Splitter Controller : ";
                    switch (data[1])
                    {
                        case 'U':
                            str_str << "Motor " << data[2] << " unreachable" << std::endl; break;
                        case 'L':
                            str_str << "Motor " << data[2] << " got lost" << std::endl; break;
                        default:
                            break;
                    }
                    break;
                default:
                    std::cout << "Received misformatted packet" << std::endl;
                    return;
            }
        }
        catch (...)
        {
            std::cout << "Received misformatted packet" << std::endl;
            return;
        }

        std::cout << str_str.str();
        std::cout.flush();//Flush screen buffer
    }

    std::array<ElectrostaticCoronaSeparator::splitter_t, 3> splitter_old = { 1,1,1 };


    void writeString(std::stringstream& string_out)
    {
        string_out << "\n";
        auto prova = string_out.str();
        serial_comm.writeString(prova);
    }

    int ack_to_receive = 0;
    CallbackAsyncSerial serial_comm = { "COM15", 115200 };


};




#endif