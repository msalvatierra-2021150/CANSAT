// Force compiler to not add padding bytes
#pragma pack(push, 1)

// The packet sent FROM the satellite/transmitter TO the ground
struct ImageChunkPacket {
  uint8_t magic1;        // E.g., 0xBE (to distinguish from telemetry 0xCA)
  uint8_t magic2;        // E.g., 0xEF
  uint8_t image_id;      // Rolls over every new picture
  uint16_t total_chunks; // Total pieces
  uint16_t chunk_index;  // Current piece (0 to total_chunks - 1)
  uint8_t payload[200];  // The actual JPEG bytes
};

// The request sent FROM the ground TO the satellite/transmitter
struct NackPacket {
  uint8_t magic1;              // E.g., 0xBA
  uint8_t magic2;              // E.g., 0xDB
  uint8_t image_id;            // Which image this NACK belongs to
  uint8_t missing_count;       // How many chunks are in the array below
  uint16_t missing_chunks[20]; // Array of chunk indexes we need resent
};

#pragma pack(pop)