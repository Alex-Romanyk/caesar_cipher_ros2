#include <iostream>
#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "caesar_cipher/msg/cipher_message.hpp"
#include "caesar_cipher/srv/cipher_answer.hpp"

#define ALPHABET_LENGTH 26


class Encoder : public rclcpp::Node {
public:
    Encoder() : Node("encoder") {
        publisher_ = this->create_publisher<caesar_cipher::msg::CipherMessage>("cipher_topic", 10);
        service_ = this->create_service<caesar_cipher::srv::CipherAnswer>(
            "check_cipher_answer", std::bind(&Encoder::check_decrypted_cipher, this, std::placeholders::_1, std::placeholders::_2));
        std::string cipher;
        int key;
        std::cout << "Enter message: ";
        std::getline(std::cin, plaintext_);
        std::cout << "Enter key: ";
        std::cin >> key;
        cipher = encrypt(plaintext_, key);

        auto msg = caesar_cipher::msg::CipherMessage();
        msg.message = cipher;
        msg.key = (int8_t)key;
        RCLCPP_INFO(this->get_logger(), "Published encrypted message: '%s' with key %d", msg.message.c_str(), msg.key);
        publisher_->publish(msg);
    }

private:
    std::string encrypt(std::string plaintext, int key) {
        std::string cipher = "";
        for (int i = 0; i < (int)plaintext.length(); i++) {
            char letter = plaintext.at(i);
            if (isalpha(letter)) {
                char offset = isupper(letter) ? 'A' : 'a';
                letter = (letter - offset + key) % ALPHABET_LENGTH + offset; 
            }
            cipher += letter;
        }
        return cipher;
    }

    void check_decrypted_cipher(const std::shared_ptr<caesar_cipher::srv::CipherAnswer::Request> request,
                                std::shared_ptr<caesar_cipher::srv::CipherAnswer::Response> response) {
        response->result = false;
        if (request->answer == plaintext_) {
            response->result = true;
        }
        RCLCPP_INFO(this->get_logger(), "Received decrypted cipher: '%s'", request->answer.c_str());
        RCLCPP_INFO(this->get_logger(), "Responding with: '%s'", response->result ? "true" : "false");
    }
    rclcpp::Publisher<caesar_cipher::msg::CipherMessage>::SharedPtr publisher_;
    rclcpp::Service<caesar_cipher::srv::CipherAnswer>::SharedPtr service_;
    std::string plaintext_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Encoder>());
    rclcpp::shutdown();
    return 0;
}

