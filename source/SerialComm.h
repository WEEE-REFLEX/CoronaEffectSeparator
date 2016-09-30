#ifndef SERIALCOMM_H
#define SERIALCOMM_H
#include <string>
#include "AsyncSerial.h"
#include <mutex>
#include <future>

/*
<SS>: from A (1st splitter) to Z (26th splitter)
each command is followed by a LF character '\n'

**** from PC to controller ****
PX<SS><float>: set positions of the splitter at X = <float>; e.g. XA2.34 --> place the 1st splitter at position X = 2.34
PY<SS><float>: guess what...
HB<T|F>: will ask for ACK at every command
SSS: motor hard stop
HHH: motor soft stop
Q<X|Y><SS>: query motor position X or Y or both position; e.g. QXA --> the controller should return RXA2.34
QQX: query all X motor positions; e.g. the controller will burst all RXA2.34	RXB1.73    RXC3.47
QQY: query all Y motor positions
QQQ: query all positions

**** from controller to PC ****
ACK: acknownledge from controller
R<X|Y><SS><float>: motor <SS> has <X|Y> position <float>; e.g. RXC3.47 --> the 3rd motor is in X position 3.47
E<error_type><SS>
I<info_type><SS>

error_type = {U: unreachable, L: got lost :-)}
info_type = {H: halted, S: stopped, O: overcurrent, R: reset to zero position}
*/


class EcsSerialCommunicator : public boost::noncopyable
{
private:
    std::thread sending_thread;
    std::list<std::string> message_queue;
    std::mutex sending_daemon_keep_alive;
    std::mutex waiting_for_ack;
    std::mutex operating_on_queue;
    std::condition_variable ack_signal;
    bool ack_received = false;

    bool wait_for_ACK = true;
    bool sending_message = false;
    CallbackAsyncSerial serial_comm = { "COM15", 115200 };


public:
    EcsSerialCommunicator(std::string serial_port_name = "COM15", unsigned long baud_rate_sel = 115200)
        :serial_comm(serial_port_name, baud_rate_sel)
    {
        splitter_old.assign({ -1, -1 });
        if (!serial_comm.isOpen())
            std::cout << "Warning: serial port " << serial_port_name << "cannot be opened." << std::endl
            << "Please provide another serial port with SetSerial ASAP." << std::endl;

        serial_comm.setCallback(boost::bind(&EcsSerialCommunicator::onSerialReceive, this, _1, _2));

        std::thread t(std::bind(&EcsSerialCommunicator::tx_daemon,this));
        sending_thread.swap(t);
    }

    ~EcsSerialCommunicator()
    {
        sending_daemon_keep_alive.lock();
        sending_thread.join();
    }

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

    void SendCommand(message_output_t command_to_be_sent, int motor_id = -1)
    {

        std::stringstream to_be_sent;
        switch (command_to_be_sent)
        {
            case message_output_t::MOTOR_HALT: to_be_sent << "HHH"; break;
            case message_output_t::MOTOR_STOP: to_be_sent << "SSS"; break;
            case message_output_t::RETRIEVE_POSX: to_be_sent << "QQX"; break;
            case message_output_t::RETRIEVE_POSY: to_be_sent << "QQY"; break;
            case message_output_t::RETRIEVE_POS: to_be_sent << "QQQ"; break;
            default: break;
        }

        sendCommand(to_be_sent);
    }


    void UpdateMotorPosition(std::array<ElectrostaticCoronaSeparator::splitter_t, 3>& splitter)
    {
        std::stringstream to_be_sent_stream;
        //to_be_sent_stream << std::setprecision(2);

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
                to_be_sent_stream << round(splitter[split_sel].pos_x * 1000);
                sendCommand(to_be_sent_stream);
                to_be_sent_stream.str(std::string());
            }

            // update height of the splitter
            if (abs(splitter[split_sel].pos_y - splitter_old[split_sel].pos_y) > 1e-2)
            {
                splitter_old[split_sel].pos_y = splitter[split_sel].pos_y;
                to_be_sent_stream << "Y" << splitter_prefix;
                to_be_sent_stream << round(splitter[split_sel].pos_y * 1000);
                sendCommand(to_be_sent_stream);
                to_be_sent_stream.str(std::string());
            }

        }

    }


private:
    void onSerialReceive(const char* data, int data_size) {

        while (data_size >= 0 && data[data_size - 1] == '\n' || data[data_size - 1] == '\r')
            --data_size;

        if (data_size < 3)
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
                    str_str << "Motor " << data[2] << " in position " << data[1] << ": " << rec_str.substr(3, rec_str.size() - 2) << "mm" << std::endl;
                    //switch (data[1])
                    //{
                    //    case 'A':
                    //        splitter_prefix = "A"; break;
                    //    case 'B':
                    //        splitter_prefix = "B"; break;
                    //    case 'C':
                    //        splitter_prefix = "C"; break;
                    //    default:
                    //        break;
                    //}
                    //break;
                case 'A':
                    if (data[1] == 'C' && data[2] == 'K')
                    {
                        if (wait_for_ACK)
                        {
                            std::lock_guard<std::mutex> temp {waiting_for_ack}; // not needed actually if no data has to be modified
                            ack_received = true;
                            ack_signal.notify_one();
                        }
                        else
                            str_str << "ACK" << std::endl;

                    }
                    break;
                case 'I':
                    str_str << "INFO from Splitter Controller : ";
                    switch (data[1])
                    {
                        case 'H':
                            str_str << "Motor " << data[2] << " halted" << std::endl; break;
                        case 'S':
                            str_str << "Motor " << data[2] << " stopped" << std::endl; break;
                        case 'R':
                            str_str << "Motor " << data[2] << " reset" << std::endl; break;
                        case 'O':
                            str_str << "Motor " << data[2] << " is asking too much current" << std::endl; break;
                        default:
                            break;
                    }
                    break;
                case 'E':
                    str_str << "Error from Splitter Controller : ";
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

        std::cout << "Received: "<< str_str.str();
        std::cout.flush();//Flush screen buffer
    }

    std::array<ElectrostaticCoronaSeparator::splitter_t, 3> splitter_old = { 1,1,1 };


    void sendCommand(std::stringstream& string_out)
    {
        string_out << "\n";
        auto string_send = string_out.str();
        if (wait_for_ACK)
        {
            std::lock_guard<std::mutex>{operating_on_queue};
            message_queue.push_back(string_send);
            std::cout << "Pushing command:" << string_send;
            //std::async(std::launch::async, std::bind(&EcsSerialCommunicator::writeQueue, this, string_send));
        }
        else
            serial_comm.writeString(string_send);
    }

    void tx_daemon()
    {
        std::cout << "Started sending daemon." << std::endl;
        while(sending_daemon_keep_alive.try_lock())
        {
            sending_daemon_keep_alive.unlock();

            operating_on_queue.lock();
            if (!message_queue.empty())
            {

                std::string local_string = *message_queue.cbegin();
                operating_on_queue.unlock();
                std::cout << "Daemon sending: " << local_string << std::endl;
                serial_comm.writeString(local_string);
                auto tx_max_attempts = 3;
                std::unique_lock<std::mutex> lock_for_ack(waiting_for_ack);
                ack_received = false;
                //sending_message = true; // can be removed if we are sure that notifying before actually waiting, does not trigger the wait
                for (auto retry_sel = 0; retry_sel < tx_max_attempts; retry_sel++)
                {
                    // we cannot rely on wait_for timer!!!! need to use additional variable
                    auto ack_signal_return = ack_signal.wait_for(lock_for_ack, std::chrono::milliseconds(3000), [=](){return ack_received;});
                    if (!ack_signal_return)
                    {
                        std::cout << "No ACK received for: " << local_string.substr(0, local_string.size()-1) << "; attempt " << retry_sel + 1 << " of " << tx_max_attempts << std::endl;
                    }
                    else
                    {
                        std::cout << "Daemon succesfully sent: " << local_string.substr(0, local_string.size() - 1) << std::endl;
                        break;
                    }
                    std::lock_guard<std::mutex> temp{ operating_on_queue };
                    message_queue.pop_front();
                }
                //sending_message = false;
            }
            else
            {
                operating_on_queue.unlock();
            }
            
        }
    }

};




#endif