#include <iostream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "caesar_cipher/msg/cipher_message.hpp"
#include "caesar_cipher/srv/cipher_answer.hpp"

#define ALPHABET_LENGTH 26
class Decoder : public rclcpp::Node {
public:
    Decoder() : Node("decoder") {
        subscription_ = this->create_subscription<caesar_cipher::msg::CipherMessage>(
            "cipher_topic", 10, std::bind(&Decoder::topic_callback, this, std::placeholders::_1));
        client_ = this->create_client<caesar_cipher::srv::CipherAnswer>("check_cipher_answer");
        RCLCPP_INFO(this->get_logger(), "Decoder started. Listening for encrypted message.");
    }
private:
    void topic_callback(const caesar_cipher::msg::CipherMessage::SharedPtr msg) {
        std::string plaintext = decrypt(msg->message, msg->key);
        RCLCPP_INFO(this->get_logger(), "Received encrypted message: '%s' with key %d", msg->message.c_str(), msg->key);
        RCLCPP_INFO(this->get_logger(), "Decrypted message: '%s'", plaintext.c_str());
        check_decrypted_cipher(plaintext);
    }

    std::string decrypt(std::string cipher, int8_t key) {
        std::string plaintext = "";
        for (int i = 0; i < (int)cipher.length(); i++) {
            char letter = cipher.at(i);
            if (isalpha(letter)) {
                char offset = isupper(letter) ? 'A' : 'a';
                letter = (letter - offset - (int)key + ALPHABET_LENGTH) % ALPHABET_LENGTH + offset;
            }
            plaintext += letter;
        }
        return plaintext;
    }

    //callback method
    void output_result(rclcpp::Client<caesar_cipher::srv::CipherAnswer>::SharedFuture future) {
        RCLCPP_INFO(this->get_logger(), "The decrypted cipher was: '%s'", future.get()->result ? "correct" : "incorrect");
    }

    void check_decrypted_cipher(std::string plaintext) {
        auto request = std::make_shared<caesar_cipher::srv::CipherAnswer::Request>();
        request->answer = plaintext;
        auto response = client_->async_send_request(request, std::bind(&Decoder::output_result, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Calling service...");
    }
    rclcpp::Subscription<caesar_cipher::msg::CipherMessage>::SharedPtr subscription_;
    rclcpp::Client<caesar_cipher::srv::CipherAnswer>::SharedPtr client_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Decoder>());
    rclcpp::shutdown();
    return 0;
}