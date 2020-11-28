#define RXD2 16
#define TXD2 17


struct dataLiDAR {
  uint8_t start = 0xFA;
  uint8_t index = 0;
  uint8_t speed_L = 0;
  uint8_t speed_H = 0;
  uint8_t data_0[4] = {0, 0, 0, 0};
  uint8_t data_1[4] = {0, 0, 0, 0};
  uint8_t data_2[4] = {0, 0, 0, 0};
  uint8_t data_3[4] = {0, 0, 0, 0};

  uint8_t checksum_L = 0;
  uint8_t checksum_H = 0;

  int angle = 0;

  void set_byte(uint8_t _byte_number, uint8_t hex) {
    if (_byte_number == 1) index = hex;
    else if (_byte_number == 2) speed_L = hex;
    else if (_byte_number == 3) speed_H = hex;

    else if (_byte_number == 4) data_0[0] = hex;
    else if (_byte_number == 5) data_1[0] = hex;
    else if (_byte_number == 6) data_2[0] = hex;
    else if (_byte_number == 7) data_3[0] = hex;

    else if (_byte_number == 8) data_0[1] = hex;
    else if (_byte_number == 9) data_1[1] = hex;
    else if (_byte_number == 10) data_2[1] = hex;
    else if (_byte_number == 11) data_3[1] = hex;

    else if (_byte_number == 12) data_0[2] = hex;
    else if (_byte_number == 13) data_1[2] = hex;
    else if (_byte_number == 14) data_2[2] = hex;
    else if (_byte_number == 15) data_3[2] = hex;

    else if (_byte_number == 16) data_0[3] = hex;
    else if (_byte_number == 17) data_1[3] = hex;
    else if (_byte_number == 18) data_2[3] = hex;
    else if (_byte_number == 19) data_3[3] = hex;

    else if (_byte_number == 20) checksum_L = hex;
    else if (_byte_number == 21) checksum_H = hex;
    else return;
    _byte_number++;
  }

  uint8_t get_byte(uint8_t byte_id) {
    if (byte_id == 0) return start;
    else if (byte_id == 1) return index;
    else if (byte_id == 2) return speed_L;
    else if (byte_id == 3) return speed_H;

    else if (byte_id == 4) return data_0[0];
    else if (byte_id == 5) return data_1[0];
    else if (byte_id == 6) return data_2[0];
    else if (byte_id == 7) return data_3[0];

    else if (byte_id == 8) return data_0[1];
    else if (byte_id == 9) return data_1[1];
    else if (byte_id == 10) return data_2[1];
    else if (byte_id == 11) return data_3[1];

    else if (byte_id == 12) return data_0[2];
    else if (byte_id == 13) return data_1[2];
    else if (byte_id == 14) return data_2[2];
    else if (byte_id == 15) return data_3[2];

    else if (byte_id == 16) return data_0[3];
    else if (byte_id == 17) return data_1[3];
    else if (byte_id == 18) return data_2[3];
    else if (byte_id == 19) return data_3[3];

    else if (byte_id == 20) return checksum_L;
    else if (byte_id == 21) return checksum_H;
    else return 0;
  }

  float rpm() {
    return float(speed_L | ((speed_H << 8))) / 64.f;
  }
  int dist_mm(uint8_t chunk) {
    return data_0[chunk] | (( data_1[chunk] & 0x3F) << 8); // 14 bits for the distance
  };

  bool verify_packet_checksum() { // 22 bytes in the packet
    int checksum32 = 0;
    for (int i = 0; i < 10; i++)
      checksum32 = (checksum32 << 1) + get_byte(2 * i) + (get_byte(2 * i + 1) << 8);
    return checksum_L + (checksum_H << 8) == (((checksum32 & 0x7FFF) + (checksum32 >> 15)) & 0x7FFF);
  }

};

uint8_t buf[30];

//int count_errors(uint8_t *buf) { // 1980 bytes in the buffer (90 packets)
//  int nb_err = 0;
//  for (int i = 0; i < 90; i++) {
//    nb_err += !verify_packet_checksum(buf + i * 22);
//  }
//  return nb_err;
//}
//
//// no return/max range/too low of reflectivity
//bool invalid_data_flag(uint8_t *data) { // 4 bytes in the data buffer
//  return (data[1] & 0x80) >> 7;
//}
//
//// object too close, possible poor reading due to proximity; kicks in at < 0.6m
//bool strength_warning_flag(uint8_t *data) { // 4 bytes in the data buffer
//  return (data[1] & 0x40) >> 6;
//}
//
//int dist_mm(uint8_t *data) { // 4 bytes in the data buffer
//  return data[0] | (( data[1] & 0x3F) << 8); // 14 bits for the distance
//}
//
//int signal_strength(uint8_t *data) { // 4 bytes in the data buffer
//  return data[2] | (data[3] << 8); // 16 bits for the signal strength
//}
//


void parse_data(uint8_t * buffer, dataLiDAR * data_parse) {
  int8_t _data_id = -1;
  uint8_t byte_id = 0;
  for (int b = 0; b < 1980; b++) {
    if (buffer[b] == 0xFA) {
      if (_data_id > -1) {
        if (byte_id < 22) {
          data_parse->set_byte(20 , data_parse->get_byte(byte_id - 2));
          data_parse->set_byte(21 , data_parse->get_byte(byte_id - 1));

          data_parse->set_byte(byte_id - 2, 0);
          data_parse->set_byte(byte_id - 1, 0);
        }
        data_parse->angle = data_parse->get_byte(1);

        Serial.print("package: ");
        for (int i = 0; i < 22; i++) {
          Serial.print(data_parse->get_byte(i), HEX);
          Serial.print(" ");
        }
        Serial.println();
        Serial.print("Valid data: ");
        if (data_parse->verify_packet_checksum())
          Serial.println("Yes");
        else
          Serial.println("No");
        return;
      }
      _data_id++;
      byte_id = 0;
    }
    data_parse->set_byte(byte_id , buffer[b]);
    byte_id++;
  }
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
}

bool read_stream(HardwareSerial *_stream, uint8_t * buffer, int count) {
  if (_stream->available()) {
    for (int elem = 0; elem < count; elem++) {
      buffer[elem] = byte(_stream->read());
    }
    return 1;
  }
  return 0;
}

void loop() {
  dataLiDAR data_l;
  if ( read_stream(&Serial2, buf, 1)) {
    if (buf[0] == 0xFA) {
      Serial.println("[========== Get new data ==========]");
      read_stream(&Serial2, buf + 1, 22);
      parse_data(buf, &data_l);


      Serial.print("#rpm: "); Serial.print(data_l.rpm()); Serial.println();
      for (int i = 0; i < 4; i++) {
        Serial.print("angle: "); Serial.print(data_l.angle + i); Serial.print("°   distance: "); Serial.print(data_l.dist_mm(i)); Serial.println(" mm");
      }

      //      for (int i = 0; i < 100; i++) Serial.print("#");
      //      for (int i = 0; i < 40; i++) Serial.println();
      //print_data(buf);
    }
  }

  //delete data_l;

}
