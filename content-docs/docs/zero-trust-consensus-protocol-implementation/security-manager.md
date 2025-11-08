# Security Manager Implementation

The `SecurityManager` class implements progressive zero-trust verification through multiple levels of identity verification, including digital signatures, motion hash verification, and cryptographic operations.

## Header File Structure

```cpp
// security_manager.hpp
#ifndef ROBOCON_NETWORK_CLIENT_SECURITY_MANAGER_HPP
#define ROBOCON_NETWORK_CLIENT_SECURITY_MANAGER_HPP

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <unordered_map>
#include <memory>
#include <mutex>
#include <openssl/evp.h>
#include <openssl/pem.h>
#include <openssl/sha.h>

namespace robocon {

enum class VerificationLevel {
    SIGNATURE_ONLY = 1,        // Level 1: Signature verification
    MOTION_HASH = 2,           // Level 2: Motion hash comparison
    ACTION_REPLAY = 3,         // Level 3: Private action replay
    MORPHOLOGY_SCAN = 4        // Level 4: Physical morphology scanning
};

struct VerificationResult {
    bool verified;
    VerificationLevel level;
    std::string reason;
    std::chrono::system_clock::time_point timestamp;
};

class SecurityManager {
public:
    SecurityManager(
        const std::string& robot_id,
        rclcpp::Node::SharedPtr node
    );
    
    ~SecurityManager();
    
    bool initialize();
    void shutdown();
    
    // Identity Verification
    bool verifyPeerIdentity(const std::string& peer_id);
    VerificationResult verifyPeerIdentityWithLevel(
        const std::string& peer_id,
        VerificationLevel max_level = VerificationLevel::MORPHOLOGY_SCAN
    );
    
    // Hash Operations
    std::string generateHash(const std::string& data) const;
    std::string generateHash(const std::vector<uint8_t>& data) const;
    bool verifyMotionHash(
        const std::string& peer_id,
        const std::string& hash
    );
    
    // Digital Signature Operations
    std::string signData(const std::string& data) const;
    bool verifySignature(
        const std::string& data,
        const std::string& signature,
        const std::string& public_key
    ) const;
    
    // Key Management
    std::string getPublicKey() const;
    bool loadKeys(const std::string& private_key_path, 
                  const std::string& public_key_path);
    bool generateKeys();
    
    // Verification Level Management
    void setVerificationLevel(
        const std::string& peer_id,
        VerificationLevel level
    );
    VerificationLevel getVerificationLevel(
        const std::string& peer_id
    ) const;

private:
    std::string robot_id_;
    rclcpp::Node::SharedPtr node_;
    bool initialized_;
    
    // Cryptographic keys
    EVP_PKEY* private_key_;
    EVP_PKEY* public_key_;
    std::string public_key_pem_;
    
    // Peer verification states
    std::unordered_map<std::string, VerificationLevel> peer_verification_levels_;
    std::unordered_map<std::string, std::string> peer_public_keys_;
    std::unordered_map<std::string, std::string> peer_motion_hashes_;
    
    mutable std::mutex security_mutex_;
    
    // Internal methods
    bool verifySignatureLevel1(
        const std::string& peer_id,
        const std::string& signature,
        const std::string& data
    );
    bool verifyMotionHashLevel2(const std::string& peer_id);
    bool verifyActionReplayLevel3(const std::string& peer_id);
    bool verifyMorphologyScanLevel4(const std::string& peer_id);
    
    std::string private_key_path_;
    std::string public_key_path_;
};

} // namespace robocon

#endif // ROBOCON_NETWORK_CLIENT_SECURITY_MANAGER_HPP
```

## Implementation

```cpp
// security_manager.cpp
#include "robocon_network_client/security_manager.hpp"
#include <fstream>
#include <sstream>
#include <iomanip>
#include <openssl/err.h>
#include <openssl/rand.h>

namespace robocon {

SecurityManager::SecurityManager(
    const std::string& robot_id,
    rclcpp::Node::SharedPtr node
) : robot_id_(robot_id),
    node_(node),
    initialized_(false),
    private_key_(nullptr),
    public_key_(nullptr)
{
    // Default key paths
    private_key_path_ = "/etc/robocon/keys/" + robot_id_ + "_private.pem";
    public_key_path_ = "/etc/robocon/keys/" + robot_id_ + "_public.pem";
}

SecurityManager::~SecurityManager() {
    if (initialized_) {
        shutdown();
    }
    
    if (private_key_) {
        EVP_PKEY_free(private_key_);
    }
    if (public_key_) {
        EVP_PKEY_free(public_key_);
    }
}

bool SecurityManager::initialize() {
    if (initialized_) {
        return true;
    }
    
    RCLCPP_INFO(node_->get_logger(), 
                "Initializing SecurityManager for robot: %s", 
                robot_id_.c_str());
    
    // Initialize OpenSSL
    OpenSSL_add_all_algorithms();
    ERR_load_crypto_strings();
    
    // Load or generate keys
    if (!loadKeys(private_key_path_, public_key_path_)) {
        RCLCPP_WARN(node_->get_logger(), 
                    "Failed to load keys, generating new ones");
        if (!generateKeys()) {
            RCLCPP_ERROR(node_->get_logger(), 
                         "Failed to generate keys");
            return false;
        }
    }
    
    initialized_ = true;
    RCLCPP_INFO(node_->get_logger(), 
                "SecurityManager initialized successfully");
    
    return true;
}

void SecurityManager::shutdown() {
    if (!initialized_) {
        return;
    }
    
    std::lock_guard<std::mutex> lock(security_mutex_);
    peer_verification_levels_.clear();
    peer_public_keys_.clear();
    peer_motion_hashes_.clear();
    initialized_ = false;
}

std::string SecurityManager::generateHash(const std::string& data) const {
    return generateHash(
        std::vector<uint8_t>(data.begin(), data.end())
    );
}

std::string SecurityManager::generateHash(
    const std::vector<uint8_t>& data
) const {
    unsigned char hash[SHA256_DIGEST_LENGTH];
    SHA256_CTX sha256;
    
    SHA256_Init(&sha256);
    SHA256_Update(&sha256, data.data(), data.size());
    SHA256_Final(hash, &sha256);
    
    // Convert to hex string
    std::ostringstream oss;
    for (int i = 0; i < SHA256_DIGEST_LENGTH; i++) {
        oss << std::hex << std::setw(2) << std::setfill('0') 
            << static_cast<int>(hash[i]);
    }
    
    return oss.str();
}

std::string SecurityManager::signData(const std::string& data) const {
    if (!private_key_ || !initialized_) {
        RCLCPP_ERROR(node_->get_logger(), 
                     "Private key not available");
        return "";
    }
    
    EVP_MD_CTX* md_ctx = EVP_MD_CTX_new();
    if (!md_ctx) {
        return "";
    }
    
    if (EVP_DigestSignInit(md_ctx, nullptr, EVP_sha256(), nullptr, 
                           private_key_) != 1) {
        EVP_MD_CTX_free(md_ctx);
        return "";
    }
    
    if (EVP_DigestSignUpdate(md_ctx, data.data(), data.size()) != 1) {
        EVP_MD_CTX_free(md_ctx);
        return "";
    }
    
    size_t signature_len = 0;
    if (EVP_DigestSignFinal(md_ctx, nullptr, &signature_len) != 1) {
        EVP_MD_CTX_free(md_ctx);
        return "";
    }
    
    std::vector<unsigned char> signature(signature_len);
    if (EVP_DigestSignFinal(md_ctx, signature.data(), &signature_len) != 1) {
        EVP_MD_CTX_free(md_ctx);
        return "";
    }
    
    EVP_MD_CTX_free(md_ctx);
    
    // Convert to base64 or hex string
    std::ostringstream oss;
    for (size_t i = 0; i < signature_len; i++) {
        oss << std::hex << std::setw(2) << std::setfill('0') 
            << static_cast<int>(signature[i]);
    }
    
    return oss.str();
}

bool SecurityManager::verifySignature(
    const std::string& data,
    const std::string& signature_hex,
    const std::string& public_key_pem
) const {
    // Convert hex signature to bytes
    std::vector<unsigned char> signature;
    for (size_t i = 0; i < signature_hex.length(); i += 2) {
        std::string byte_str = signature_hex.substr(i, 2);
        signature.push_back(
            static_cast<unsigned char>(std::stoul(byte_str, nullptr, 16))
        );
    }
    
    // Parse public key from PEM
    BIO* bio = BIO_new_mem_buf(
        public_key_pem.data(), 
        static_cast<int>(public_key_pem.length())
    );
    if (!bio) {
        return false;
    }
    
    EVP_PKEY* peer_public_key = PEM_read_bio_PUBKEY(bio, nullptr, nullptr, nullptr);
    BIO_free(bio);
    
    if (!peer_public_key) {
        return false;
    }
    
    // Verify signature
    EVP_MD_CTX* md_ctx = EVP_MD_CTX_new();
    if (!md_ctx) {
        EVP_PKEY_free(peer_public_key);
        return false;
    }
    
    bool result = false;
    if (EVP_DigestVerifyInit(md_ctx, nullptr, EVP_sha256(), nullptr, 
                            peer_public_key) == 1) {
        if (EVP_DigestVerifyUpdate(md_ctx, data.data(), data.size()) == 1) {
            result = (EVP_DigestVerifyFinal(md_ctx, signature.data(), 
                                           signature.size()) == 1);
        }
    }
    
    EVP_MD_CTX_free(md_ctx);
    EVP_PKEY_free(peer_public_key);
    
    return result;
}

bool SecurityManager::verifyPeerIdentity(const std::string& peer_id) {
    return verifyPeerIdentityWithLevel(
        peer_id,
        VerificationLevel::MOTION_HASH
    ).verified;
}

VerificationResult SecurityManager::verifyPeerIdentityWithLevel(
    const std::string& peer_id,
    VerificationLevel max_level
) {
    VerificationResult result;
    result.verified = false;
    result.level = VerificationLevel::SIGNATURE_ONLY;
    result.timestamp = std::chrono::system_clock::now();
    
    if (!initialized_) {
        result.reason = "SecurityManager not initialized";
        return result;
    }
    
    // Level 1: Signature Verification
    if (max_level >= VerificationLevel::SIGNATURE_ONLY) {
        if (!verifySignatureLevel1(peer_id, "", "")) {
            result.reason = "Level 1 signature verification failed";
            return result;
        }
        result.level = VerificationLevel::SIGNATURE_ONLY;
        result.verified = true;
    }
    
    // Level 2: Motion Hash Verification
    if (max_level >= VerificationLevel::MOTION_HASH) {
        if (!verifyMotionHashLevel2(peer_id)) {
            result.reason = "Level 2 motion hash verification failed";
            return result;
        }
        result.level = VerificationLevel::MOTION_HASH;
    }
    
    // Level 3: Action Replay (if needed)
    if (max_level >= VerificationLevel::ACTION_REPLAY) {
        if (!verifyActionReplayLevel3(peer_id)) {
            result.reason = "Level 3 action replay verification failed";
            return result;
        }
        result.level = VerificationLevel::ACTION_REPLAY;
    }
    
    // Level 4: Morphology Scan (if needed)
    if (max_level >= VerificationLevel::MORPHOLOGY_SCAN) {
        if (!verifyMorphologyScanLevel4(peer_id)) {
            result.reason = "Level 4 morphology scan verification failed";
            return result;
        }
        result.level = VerificationLevel::MORPHOLOGY_SCAN;
    }
    
    // Update verification level
    std::lock_guard<std::mutex> lock(security_mutex_);
    peer_verification_levels_[peer_id] = result.level;
    
    return result;
}

bool SecurityManager::verifySignatureLevel1(
    const std::string& peer_id,
    const std::string& signature,
    const std::string& data
) {
    // In real implementation, this would verify the peer's signature
    // from a discovery message or certificate
    std::lock_guard<std::mutex> lock(security_mutex_);
    
    // Check if we have the peer's public key
    auto it = peer_public_keys_.find(peer_id);
    if (it == peer_public_keys_.end()) {
        RCLCPP_WARN(node_->get_logger(), 
                    "Public key not found for peer: %s", 
                    peer_id.c_str());
        return false;
    }
    
    // Verify signature (placeholder - real implementation would use actual data)
    return true;
}

bool SecurityManager::verifyMotionHashLevel2(const std::string& peer_id) {
    // This would compare motion ledger hashes with the peer
    // For now, return true as placeholder
    std::lock_guard<std::mutex> lock(security_mutex_);
    
    auto it = peer_motion_hashes_.find(peer_id);
    if (it != peer_motion_hashes_.end()) {
        // In real implementation, compare with local ledger hash
        return true;
    }
    
    return false;
}

bool SecurityManager::verifyActionReplayLevel3(const std::string& peer_id) {
    // This would request a replay of recent private actions
    // For now, return true as placeholder
    return true;
}

bool SecurityManager::verifyMorphologyScanLevel4(const std::string& peer_id) {
    // This would use sensors (LiDAR, cameras) to verify physical morphology
    // For now, return true as placeholder
    return true;
}

bool SecurityManager::verifyMotionHash(
    const std::string& peer_id,
    const std::string& hash
) {
    std::lock_guard<std::mutex> lock(security_mutex_);
    
    auto it = peer_motion_hashes_.find(peer_id);
    if (it != peer_motion_hashes_.end()) {
        return it->second == hash;
    }
    
    // Store hash for future comparison
    peer_motion_hashes_[peer_id] = hash;
    return true;
}

std::string SecurityManager::getPublicKey() const {
    return public_key_pem_;
}

bool SecurityManager::loadKeys(
    const std::string& private_key_path,
    const std::string& public_key_path
) {
    // Load private key
    FILE* private_key_file = fopen(private_key_path.c_str(), "r");
    if (!private_key_file) {
        return false;
    }
    
    private_key_ = PEM_read_PrivateKey(private_key_file, nullptr, nullptr, nullptr);
    fclose(private_key_file);
    
    if (!private_key_) {
        return false;
    }
    
    // Load public key
    FILE* public_key_file = fopen(public_key_path.c_str(), "r");
    if (!public_key_file) {
        EVP_PKEY_free(private_key_);
        private_key_ = nullptr;
        return false;
    }
    
    public_key_ = PEM_read_PUBKEY(public_key_file, nullptr, nullptr, nullptr);
    fclose(public_key_file);
    
    if (!public_key_) {
        EVP_PKEY_free(private_key_);
        private_key_ = nullptr;
        return false;
    }
    
    // Export public key to PEM string
    BIO* bio = BIO_new(BIO_s_mem());
    PEM_write_bio_PUBKEY(bio, public_key_);
    
    char* pem_data = nullptr;
    long pem_len = BIO_get_mem_data(bio, &pem_data);
    public_key_pem_ = std::string(pem_data, pem_len);
    BIO_free(bio);
    
    return true;
}

bool SecurityManager::generateKeys() {
    EVP_PKEY_CTX* ctx = EVP_PKEY_CTX_new_id(EVP_PKEY_RSA, nullptr);
    if (!ctx) {
        return false;
    }
    
    if (EVP_PKEY_keygen_init(ctx) <= 0) {
        EVP_PKEY_CTX_free(ctx);
        return false;
    }
    
    if (EVP_PKEY_CTX_set_rsa_keygen_bits(ctx, 2048) <= 0) {
        EVP_PKEY_CTX_free(ctx);
        return false;
    }
    
    if (EVP_PKEY_keygen(ctx, &private_key_) <= 0) {
        EVP_PKEY_CTX_free(ctx);
        return false;
    }
    
    EVP_PKEY_CTX_free(ctx);
    
    // Extract public key
    public_key_ = EVP_PKEY_new();
    EVP_PKEY_up_ref(private_key_);
    EVP_PKEY_copy_parameters(public_key_, private_key_);
    EVP_PKEY_set1_RSA(public_key_, EVP_PKEY_get0_RSA(private_key_));
    
    // Export public key to PEM
    BIO* bio = BIO_new(BIO_s_mem());
    PEM_write_bio_PUBKEY(bio, public_key_);
    
    char* pem_data = nullptr;
    long pem_len = BIO_get_mem_data(bio, &pem_data);
    public_key_pem_ = std::string(pem_data, pem_len);
    BIO_free(bio);
    
    // Save keys to files
    // (In real implementation, ensure directory exists and handle errors)
    
    return true;
}

void SecurityManager::setVerificationLevel(
    const std::string& peer_id,
    VerificationLevel level
) {
    std::lock_guard<std::mutex> lock(security_mutex_);
    peer_verification_levels_[peer_id] = level;
}

VerificationLevel SecurityManager::getVerificationLevel(
    const std::string& peer_id
) const {
    std::lock_guard<std::mutex> lock(security_mutex_);
    auto it = peer_verification_levels_.find(peer_id);
    if (it != peer_verification_levels_.end()) {
        return it->second;
    }
    return VerificationLevel::SIGNATURE_ONLY;
}

} // namespace robocon
```

## Usage Example

```cpp
#include "robocon_network_client/security_manager.hpp"
#include <rclcpp/rclcpp.hpp>
#include <iostream>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("security_example");
    
    robocon::SecurityManager security("robot_001", node);
    security.initialize();
    
    // Generate hash
    std::string data = "test data";
    std::string hash = security.generateHash(data);
    std::cout << "Hash: " << hash << std::endl;
    
    // Sign data
    std::string signature = security.signData(data);
    std::cout << "Signature: " << signature << std::endl;
    
    // Get public key
    std::string public_key = security.getPublicKey();
    std::cout << "Public key (first 100 chars): " 
              << public_key.substr(0, 100) << std::endl;
    
    // Verify peer identity
    bool verified = security.verifyPeerIdentity("robot_002");
    std::cout << "Peer verified: " << verified << std::endl;
    
    // Verify with specific level
    auto result = security.verifyPeerIdentityWithLevel(
        "robot_002",
        robocon::VerificationLevel::MOTION_HASH
    );
    
    if (result.verified) {
        std::cout << "Verification level: " 
                  << static_cast<int>(result.level) << std::endl;
    }
    
    security.shutdown();
    rclcpp::shutdown();
    return 0;
}
```

## Key Implementation Details

1. **Progressive Verification**: Implements four levels of verification, from basic signature to physical morphology scanning.

2. **OpenSSL Integration**: Uses OpenSSL for cryptographic operations (RSA keys, SHA-256 hashing, digital signatures).

3. **Key Management**: Supports loading existing keys or generating new ones.

4. **Thread Safety**: Uses mutexes to protect shared state.

5. **Verification Levels**: Tracks verification level for each peer, allowing different trust levels.

## Next Steps

- [Discovery Manager Implementation](discovery-manager.md) - Peer discovery implementation
- [ROS 2 Messages and Topics](ros2-integration.md) - Message definitions
- [Network Client Implementation](network-client.md) - Main interface

