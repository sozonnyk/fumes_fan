// https://pionierland.de/fonts/index.php

extern const uint8_t font[] PROGMEM;
const uint8_t font[] PROGMEM =
{
//  type|width|height|first char
    0x02, 0x25, 0x18, 0x00,
// GROUP first ' ' total 96 chars
//  unicode(LSB,MSB)|count
    0x00, 0x20, 0x60, // unicode record
    0x00, 0x00, 0x01, 0x00,// char ' ' (0x0020/32)
    0x00, 0x00, 0x04, 0x18,// char '!' (0x0021/33)
    0x00, 0x0C, 0x06, 0x0C,// char '"' (0x0022/34)
    0x00, 0x18, 0x12, 0x18,// char '#' (0x0023/35)
    0x00, 0x4E, 0x0E, 0x18,// char '$' (0x0024/36)
    0x00, 0x78, 0x13, 0x18,// char '%' (0x0025/37)
    0x00, 0xB1, 0x11, 0x18,// char '&' (0x0026/38)
    0x00, 0xE4, 0x02, 0x0C,// char ''' (0x0027/39)
    0x00, 0xE8, 0x07, 0x18,// char '(' (0x0028/40)
    0x00, 0xFD, 0x07, 0x18,// char ')' (0x0029/41)
    0x01, 0x12, 0x08, 0x0E,// char '*' (0x002A/42)
    0x01, 0x22, 0x0D, 0x14,// char '+' (0x002B/43)
    0x01, 0x49, 0x04, 0x18,// char ',' (0x002C/44)
    0x01, 0x55, 0x09, 0x11,// char '-' (0x002D/45)
    0x01, 0x70, 0x04, 0x18,// char '.' (0x002E/46)
    0x01, 0x7C, 0x0E, 0x18,// char '/' (0x002F/47)
    0x01, 0xA6, 0x0F, 0x18,// char '0' (0x0030/48)
    0x01, 0xD3, 0x07, 0x18,// char '1' (0x0031/49)
    0x01, 0xE8, 0x0D, 0x18,// char '2' (0x0032/50)
    0x02, 0x0F, 0x0D, 0x18,// char '3' (0x0033/51)
    0x02, 0x36, 0x0D, 0x18,// char '4' (0x0034/52)
    0x02, 0x5D, 0x0D, 0x18,// char '5' (0x0035/53)
    0x02, 0x84, 0x0D, 0x18,// char '6' (0x0036/54)
    0x02, 0xAB, 0x0E, 0x18,// char '7' (0x0037/55)
    0x02, 0xD5, 0x0E, 0x18,// char '8' (0x0038/56)
    0x02, 0xFF, 0x0E, 0x18,// char '9' (0x0039/57)
    0x03, 0x29, 0x04, 0x18,// char ':' (0x003A/58)
    0x03, 0x35, 0x04, 0x18,// char ';' (0x003B/59)
    0x03, 0x41, 0x0D, 0x15,// char '<' (0x003C/60)
    0x03, 0x68, 0x0D, 0x11,// char '=' (0x003D/61)
    0x03, 0x8F, 0x0D, 0x15,// char '>' (0x003E/62)
    0x03, 0xB6, 0x0B, 0x18,// char '?' (0x003F/63)
    0x03, 0xD7, 0x16, 0x18,// char '@' (0x0040/64)
    0x04, 0x19, 0x14, 0x18,// char 'A' (0x0041/65)
    0x04, 0x55, 0x0F, 0x18,// char 'B' (0x0042/66)
    0x04, 0x82, 0x12, 0x18,// char 'C' (0x0043/67)
    0x04, 0xB8, 0x11, 0x18,// char 'D' (0x0044/68)
    0x04, 0xEB, 0x0D, 0x18,// char 'E' (0x0045/69)
    0x05, 0x12, 0x0C, 0x18,// char 'F' (0x0046/70)
    0x05, 0x36, 0x12, 0x18,// char 'G' (0x0047/71)
    0x05, 0x6C, 0x0F, 0x18,// char 'H' (0x0048/72)
    0x05, 0x99, 0x03, 0x18,// char 'I' (0x0049/73)
    0x05, 0xA2, 0x0B, 0x18,// char 'J' (0x004A/74)
    0x05, 0xC3, 0x10, 0x18,// char 'K' (0x004B/75)
    0x05, 0xF3, 0x0B, 0x18,// char 'L' (0x004C/76)
    0x06, 0x14, 0x15, 0x18,// char 'M' (0x004D/77)
    0x06, 0x53, 0x11, 0x18,// char 'N' (0x004E/78)
    0x06, 0x86, 0x14, 0x18,// char 'O' (0x004F/79)
    0x06, 0xC2, 0x0E, 0x18,// char 'P' (0x0050/80)
    0x06, 0xEC, 0x16, 0x18,// char 'Q' (0x0051/81)
    0x07, 0x2E, 0x10, 0x18,// char 'R' (0x0052/82)
    0x07, 0x5E, 0x0F, 0x18,// char 'S' (0x0053/83)
    0x07, 0x8B, 0x0E, 0x18,// char 'T' (0x0054/84)
    0x07, 0xB5, 0x10, 0x18,// char 'U' (0x0055/85)
    0x07, 0xE5, 0x13, 0x18,// char 'V' (0x0056/86)
    0x08, 0x1E, 0x1C, 0x18,// char 'W' (0x0057/87)
    0x08, 0x72, 0x12, 0x18,// char 'X' (0x0058/88)
    0x08, 0xA8, 0x11, 0x18,// char 'Y' (0x0059/89)
    0x08, 0xDB, 0x10, 0x18,// char 'Z' (0x005A/90)
    0x09, 0x0B, 0x06, 0x18,// char '[' (0x005B/91)
    0x09, 0x1D, 0x0C, 0x18,// char '\' (0x005C/92)
    0x09, 0x41, 0x06, 0x18,// char ']' (0x005D/93)
    0x09, 0x53, 0x09, 0x04,// char '^' (0x005E/94)
    0x09, 0x5C, 0x10, 0x18,// char '_' (0x005F/95)
    0x09, 0x8C, 0x06, 0x08,// char '`' (0x0060/96)
    0x09, 0x92, 0x0D, 0x18,// char 'a' (0x0061/97)
    0x09, 0xB9, 0x0F, 0x18,// char 'b' (0x0062/98)
    0x09, 0xE6, 0x0E, 0x18,// char 'c' (0x0063/99)
    0x0A, 0x10, 0x0F, 0x18,// char 'd' (0x0064/100)
    0x0A, 0x3D, 0x0F, 0x18,// char 'e' (0x0065/101)
    0x0A, 0x6A, 0x0A, 0x18,// char 'f' (0x0066/102)
    0x0A, 0x88, 0x0E, 0x18,// char 'g' (0x0067/103)
    0x0A, 0xB2, 0x0D, 0x18,// char 'h' (0x0068/104)
    0x0A, 0xD9, 0x04, 0x18,// char 'i' (0x0069/105)
    0x0A, 0xE5, 0x09, 0x18,// char 'j' (0x006A/106)
    0x0B, 0x00, 0x0D, 0x18,// char 'k' (0x006B/107)
    0x0B, 0x27, 0x03, 0x18,// char 'l' (0x006C/108)
    0x0B, 0x30, 0x17, 0x18,// char 'm' (0x006D/109)
    0x0B, 0x75, 0x0D, 0x18,// char 'n' (0x006E/110)
    0x0B, 0x9C, 0x0F, 0x18,// char 'o' (0x006F/111)
    0x0B, 0xC9, 0x0F, 0x18,// char 'p' (0x0070/112)
    0x0B, 0xF6, 0x0F, 0x18,// char 'q' (0x0071/113)
    0x0C, 0x23, 0x08, 0x18,// char 'r' (0x0072/114)
    0x0C, 0x3B, 0x0B, 0x18,// char 's' (0x0073/115)
    0x0C, 0x5C, 0x0A, 0x18,// char 't' (0x0074/116)
    0x0C, 0x7A, 0x0D, 0x18,// char 'u' (0x0075/117)
    0x0C, 0xA1, 0x0F, 0x18,// char 'v' (0x0076/118)
    0x0C, 0xCE, 0x18, 0x18,// char 'w' (0x0077/119)
    0x0D, 0x16, 0x0E, 0x18,// char 'x' (0x0078/120)
    0x0D, 0x40, 0x0F, 0x18,// char 'y' (0x0079/121)
    0x0D, 0x6D, 0x0C, 0x18,// char 'z' (0x007A/122)
    0x0D, 0x91, 0x07, 0x18,// char '{' (0x007B/123)
    0x0D, 0xA6, 0x03, 0x18,// char '|' (0x007C/124)
    0x0D, 0xAF, 0x07, 0x18,// char '}' (0x007D/125)
    0x0D, 0xC4, 0x0D, 0x10,// char '~' (0x007E/126)
    0x0D, 0xDE, 0x01, 0x00,// char '' (0x007F/127)
    0x0D, 0xDE,
    // char ' ' (0x0020/32)
    0xE0, 0xE0, 0xE0, 0xE0, 0x1F, 0xFF, 0xFF, 0x1F, 0xE0, 0xF3, 0xF3, 0xE0, // char '!' (0x0021/33)
    0xE0, 0xE0, 0x00, 0x00, 0xE0, 0xE0, 0x0F, 0x0F, 0x00, 0x00, 0x0F, 0x0F, // char '"' (0x0022/34)
    0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0xE0, 0xE0, 0x20, 0x00, 0x00, 0x00, 0xE0, 0xE0, 0xE0, 0x20, 0x00, 0x00, 0x00, 0x0C, 0x0E, 0x0E, 0xFE, 0xFF, 0xFF, 0x0F, 0x0E, 0x0E, 0x0E, 0xFE, 0xFF, 0xFF, 0x0F, 0x0E, 0x0E, 0x02, 0x0C, 0x0E, 0xCE, 0xFF, 0xFF, 0x3F, 0x0F, 0x0E, 0x0E, 0xCE, 0xFF, 0xFF, 0x3F, 0x0E, 0x0E, 0x0E, 0x02, 0x00, // char '#' (0x0023/35)
    0x00, 0x00, 0xC0, 0xC0, 0xE0, 0xE0, 0xE0, 0xF0, 0xE0, 0xE0, 0xE0, 0xC0, 0xC0, 0x80, 0x00, 0x1F, 0x3F, 0x7F, 0x79, 0x70, 0xF0, 0xFF, 0xE0, 0xE0, 0xC0, 0xC1, 0x81, 0x80, 0x10, 0x38, 0x78, 0x70, 0xF0, 0xE0, 0xE0, 0xFF, 0xE0, 0xE1, 0xF1, 0x7F, 0x7F, 0x3F, // char '$' (0x0024/36)
    0x80, 0xC0, 0xE0, 0x60, 0x60, 0xE0, 0xE0, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xE0, 0xE0, 0x60, 0x20, 0x00, 0x07, 0x0F, 0x1F, 0x18, 0x18, 0x1C, 0x1F, 0x8F, 0xC0, 0xF0, 0xF8, 0x3E, 0x1F, 0x0F, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x80, 0xC0, 0xF0, 0xF8, 0x3C, 0x1F, 0x0F, 0x03, 0x01, 0x00, 0x3C, 0x7F, 0xFF, 0xC3, 0xC3, 0xE7, 0x7E, 0x3C, // char '%' (0x0025/37)
    0x00, 0x00, 0x00, 0x80, 0xC0, 0xE0, 0xE0, 0x60, 0x60, 0xE0, 0xE0, 0xC0, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xCF, 0xFF, 0xFF, 0x78, 0xF0, 0xF8, 0xB8, 0x1F, 0x0F, 0x07, 0x80, 0x00, 0x00, 0x00, 0x0C, 0x3F, 0x7F, 0xF3, 0xE1, 0xE0, 0xC0, 0xE1, 0xE3, 0xE7, 0x7F, 0x7E, 0x3C, 0x7F, 0xFF, 0xE3, 0x40, // char '&' (0x0026/38)
    0xE0, 0xE0, 0x0F, 0x0F, // char ''' (0x0027/39)
    0x00, 0x00, 0xC0, 0xF0, 0xF0, 0x30, 0x10, 0xE0, 0xFE, 0xFF, 0x1F, 0x01, 0x00, 0x00, 0x03, 0x7F, 0xFF, 0xFC, 0xC0, 0x00, 0x00, // char '(' (0x0028/40)
    0x10, 0x30, 0xF0, 0xF0, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x01, 0x3F, 0xFF, 0xFE, 0xC0, 0x00, 0x00, 0xC0, 0xFE, 0xFF, 0x3F, 0x03, // char ')' (0x0029/41)
    0xC0, 0x80, 0x00, 0xE0, 0xE0, 0x00, 0x80, 0xC0, 0x0C, 0x0D, 0x07, 0x3F, 0x3F, 0x05, 0x0D, 0x08, // char '*' (0x002A/42)
    0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x70, 0x70, 0x70, 0x70, 0x70, 0xFF, 0xFF, 0xFF, 0x70, 0x70, 0x70, 0x70, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x0F, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, // char '+' (0x002B/43)
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0xF0, 0xF0, 0x60, // char ',' (0x002C/44)
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, // char '-' (0x002D/45)
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0xF0, 0xF0, 0x60, // char '.' (0x002E/46)
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0xF0, 0xFC, 0x3C, 0x0C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xE0, 0xF8, 0x3F, 0x0F, 0x03, 0x00, 0x00, 0x00, 0x00, 0x80, 0xE0, 0xF8, 0x7E, 0x1F, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // char '/' (0x002F/47)
    0x00, 0x80, 0xC0, 0xC0, 0xE0, 0xE0, 0x60, 0x60, 0xE0, 0xE0, 0xE0, 0xC0, 0x80, 0x00, 0x00, 0xFE, 0xFF, 0xFF, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x07, 0xFF, 0xFF, 0xFC, 0x0F, 0x3F, 0x7F, 0x78, 0xF0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xF0, 0x7E, 0x3F, 0x1F, 0x07, // char '0' (0x0030/48)
    0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, // char '1' (0x0031/49)
    0x00, 0x80, 0xC0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xC0, 0x80, 0x00, 0x01, 0x01, 0x03, 0x01, 0x00, 0x00, 0x80, 0xC0, 0xE0, 0xF1, 0x7F, 0x3F, 0x0F, 0x00, 0xF0, 0xF8, 0xFC, 0xFE, 0xEF, 0xE7, 0xE3, 0xE1, 0xE0, 0xE0, 0xE0, 0xE0, // char '2' (0x0032/50)
    0x00, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x70, 0x78, 0x7C, 0x7F, 0xEF, 0xE7, 0xE1, 0xC0, 0x00, 0x60, 0x70, 0xF0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xF1, 0x7F, 0x3F, 0x1F, // char '3' (0x0033/51)
    0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xC0, 0xE0, 0xE0, 0x60, 0x00, 0x00, 0x00, 0x00, 0xC0, 0xF0, 0xF8, 0x7E, 0x1F, 0x0F, 0x03, 0xE0, 0xE0, 0xE0, 0x00, 0x00, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0xFF, 0xFF, 0xFF, 0x07, 0x07, // char '4' (0x0034/52)
    0x00, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0x00, 0x00, 0x3F, 0xFF, 0x7F, 0x60, 0x70, 0x70, 0x70, 0x70, 0xF0, 0xE0, 0xC0, 0x80, 0x20, 0x70, 0x78, 0xF0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xF0, 0x7F, 0x3F, 0x1F, // char '5' (0x0035/53)
    0x00, 0x80, 0xC0, 0xC0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xC0, 0x40, 0xFE, 0xFF, 0xFF, 0xE3, 0x61, 0x70, 0x70, 0x70, 0x70, 0xF0, 0xE1, 0xE0, 0x80, 0x0F, 0x3F, 0x7F, 0x78, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0x79, 0x7F, 0x1F, // char '6' (0x0036/54)
    0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0x07, 0x07, 0x07, 0x00, 0x00, 0x00, 0x80, 0xE0, 0xF8, 0xFE, 0x3F, 0x0F, 0x03, 0x00, 0x00, 0x00, 0x80, 0xE0, 0xF8, 0xFE, 0x3F, 0x0F, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, // char '7' (0x0037/55)
    0x00, 0x80, 0xC0, 0xE0, 0xE0, 0x60, 0x60, 0x60, 0xE0, 0xE0, 0xC0, 0x80, 0x00, 0x00, 0x87, 0xDF, 0xFF, 0xF9, 0xF0, 0x60, 0x60, 0x60, 0xF0, 0xF9, 0xFF, 0xDF, 0x8F, 0x00, 0x3F, 0x7F, 0x7F, 0xF0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xF0, 0x7F, 0x7F, 0x3F, 0x0E, // char '8' (0x0038/56)
    0x00, 0x80, 0xC0, 0xC0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xC0, 0xC0, 0x00, 0x00, 0x0E, 0x7F, 0xFF, 0xF3, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xE1, 0xF3, 0xFF, 0xFF, 0xFC, 0x00, 0x40, 0x70, 0xE1, 0xE1, 0xE1, 0xE1, 0xE1, 0xE1, 0xF0, 0x7C, 0x3F, 0x1F, 0x07, // char '9' (0x0039/57)
    0x00, 0x00, 0x00, 0x00, 0xC0, 0xC0, 0xC0, 0xC0, 0xE1, 0xF3, 0xF1, 0x61, // char ':' (0x003A/58)
    0x00, 0x00, 0x00, 0x00, 0xC0, 0xC0, 0xC0, 0xC0, 0xE1, 0xF3, 0xF3, 0xE1, // char ';' (0x003B/59)
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80, 0xC0, 0xC0, 0x78, 0xF8, 0xF8, 0xFC, 0xDC, 0xCE, 0x8E, 0x87, 0x07, 0x03, 0x03, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x03, 0x03, 0x07, 0x07, 0x0E, 0x0E, 0x1C, // char '<' (0x003C/60)
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x8C, 0x8C, 0x8C, 0x8C, 0x8C, 0x8C, 0x8C, 0x8C, 0x8C, 0x8C, 0x8C, 0x8C, 0x8C, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, // char '=' (0x003D/61)
    0xC0, 0xC0, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x03, 0x03, 0x07, 0x87, 0x8E, 0xCE, 0xDC, 0xFC, 0xF8, 0xF8, 0x78, 0x1C, 0x0E, 0x0E, 0x07, 0x07, 0x03, 0x03, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, // char '>' (0x003E/62)
    0x00, 0xC0, 0xC0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xC0, 0x80, 0x03, 0x03, 0x03, 0x01, 0x80, 0xE0, 0xF0, 0x78, 0x3F, 0x1F, 0x0F, 0x00, 0x00, 0x00, 0x00, 0xE3, 0xF3, 0xF3, 0x60, 0x00, 0x00, 0x00, // char '?' (0x003F/63)
    0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80, 0x80, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0xC0, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xF0, 0x1C, 0x06, 0x03, 0xE1, 0xF0, 0x78, 0x3C, 0x1C, 0x1C, 0x1C, 0x18, 0x38, 0xFC, 0xFC, 0xFD, 0x01, 0x02, 0x0C, 0x78, 0xE0, 0x07, 0x3F, 0x60, 0x80, 0x03, 0x1F, 0x3F, 0x78, 0x70, 0x60, 0x60, 0x60, 0x60, 0x30, 0x1E, 0x3F, 0x7F, 0x60, 0x40, 0x60, 0x38, 0x1F, // char '@' (0x0040/64)
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xE0, 0xE0, 0xE0, 0xE0, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0xF8, 0xFE, 0x7F, 0x0F, 0x03, 0x03, 0x0F, 0x7F, 0xFE, 0xF8, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x80, 0xE0, 0xFC, 0xFF, 0x3F, 0x0F, 0x0F, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E, 0x0F, 0x0F, 0x1F, 0xFF, 0xFC, 0xE0, 0x80, // char 'A' (0x0041/65)
    0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xC0, 0xC0, 0x80, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xF1, 0xFF, 0xFF, 0x9F, 0x06, 0x00, 0xFF, 0xFF, 0xFF, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xF1, 0x7F, 0x7F, 0x3F, 0x0E, // char 'B' (0x0042/66)
    0x00, 0x00, 0x00, 0x80, 0xC0, 0xC0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xC0, 0x80, 0x80, 0xE0, 0xFC, 0xFF, 0xFF, 0x07, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x01, 0x00, 0x00, 0x07, 0x1F, 0x3F, 0x7C, 0x78, 0xF0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xF0, 0x70, 0x78, 0x30, 0x10, // char 'C' (0x0043/67)
    0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xC0, 0xC0, 0xC0, 0x80, 0x80, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x03, 0x07, 0xFF, 0xFF, 0xFC, 0xE0, 0xFF, 0xFF, 0xFF, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0x70, 0x78, 0x7C, 0x3F, 0x1F, 0x07, 0x00, // char 'D' (0x0044/68)
    0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xFF, 0xFF, 0xFF, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0x00, 0xFF, 0xFF, 0xFF, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, // char 'E' (0x0045/69)
    0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xFF, 0xFF, 0xFF, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0x00, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // char 'F' (0x0046/70)
    0x00, 0x00, 0x00, 0x80, 0xC0, 0xC0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xC0, 0x80, 0x00, 0xE0, 0xFC, 0xFF, 0xFF, 0x07, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x81, 0x80, 0x80, 0x00, 0x07, 0x1F, 0x3F, 0x7C, 0x78, 0xF0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xF0, 0x7F, 0x7F, 0x3F, // char 'G' (0x0047/71)
    0xE0, 0xE0, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0xE0, 0xE0, 0xFF, 0xFF, 0xFF, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, // char 'H' (0x0048/72)
    0xE0, 0xE0, 0xE0, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, // char 'I' (0x0049/73)
    0x00, 0x00, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x20, 0x78, 0xF0, 0xE0, 0xE0, 0xE0, 0xE0, 0xF0, 0x7F, 0x7F, 0x3F, // char 'J' (0x004A/74)
    0xE0, 0xE0, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xC0, 0xE0, 0xE0, 0x60, 0x20, 0x20, 0xFF, 0xFF, 0xFF, 0xC0, 0xE0, 0xF8, 0xFC, 0xFE, 0xDF, 0x0F, 0x07, 0x01, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x03, 0x01, 0x00, 0x01, 0x03, 0x0F, 0x1F, 0x3E, 0xFC, 0xF0, 0xE0, 0xC0, 0x80, // char 'K' (0x004B/75)
    0xE0, 0xE0, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, // char 'L' (0x004C/76)
    0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xFF, 0xFF, 0xFF, 0x07, 0x1F, 0x7F, 0xFE, 0xF8, 0xE0, 0x80, 0x00, 0xC0, 0xF0, 0xFC, 0xFF, 0x3F, 0x0F, 0x03, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x01, 0x07, 0x1F, 0x3F, 0x3E, 0x3F, 0x0F, 0x03, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, // char 'M' (0x004D/77)
    0xE0, 0xE0, 0xE0, 0xC0, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0xE0, 0xE0, 0xFF, 0xFF, 0xFF, 0x07, 0x0F, 0x3E, 0x7C, 0xF8, 0xF0, 0xC0, 0x80, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x07, 0x0F, 0x1F, 0x7C, 0xF8, 0xFF, 0xFF, 0xFF, // char 'N' (0x004E/78)
    0x00, 0x00, 0x00, 0x80, 0xC0, 0xC0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xC0, 0xC0, 0x80, 0x00, 0x00, 0x00, 0xE0, 0xFC, 0xFF, 0xFF, 0x07, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x0F, 0xFF, 0xFE, 0xFC, 0x00, 0x07, 0x1F, 0x3F, 0x3C, 0x78, 0xF0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xF0, 0x70, 0x78, 0x3E, 0x1F, 0x0F, 0x03, // char 'O' (0x004F/79)
    0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xC0, 0xC0, 0xC0, 0x80, 0x00, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80, 0x81, 0xC3, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x01, 0x01, 0x00, // char 'P' (0x0050/80)
    0x00, 0x00, 0x00, 0x80, 0xC0, 0xC0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xC0, 0xC0, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0xFC, 0xFF, 0xFF, 0x07, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x0F, 0xFF, 0xFE, 0xFC, 0x00, 0x00, 0x00, 0x07, 0x1F, 0x3F, 0x3C, 0x78, 0xF0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xF0, 0x70, 0x78, 0x3E, 0x1F, 0x0F, 0x83, 0x00, 0x00, // char 'Q' (0x0051/81)
    0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xC0, 0xC0, 0xC0, 0x80, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0xC0, 0xC1, 0xFF, 0xFF, 0x7F, 0x1C, 0x00, 0xFF, 0xFF, 0xFF, 0x03, 0x03, 0x03, 0x03, 0x03, 0x0F, 0x1F, 0x7F, 0xFD, 0xF0, 0xE0, 0x80, 0x80, // char 'R' (0x0052/82)
    0x00, 0x80, 0xC0, 0xC0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xC0, 0xC0, 0x00, 0x00, 0x1F, 0x3F, 0x7F, 0x79, 0x70, 0xF0, 0xE0, 0xE0, 0xE0, 0xE0, 0xC1, 0xC1, 0x80, 0x00, 0x20, 0x38, 0x78, 0x70, 0xF0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE1, 0xE1, 0x7F, 0x7F, 0x3F, 0x0E, // char 'S' (0x0053/83)
    0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, // char 'T' (0x0054/84)
    0xE0, 0xE0, 0xE0, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0xE0, 0xE0, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x03, 0x1F, 0x3F, 0x7F, 0x78, 0xF0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xF0, 0x78, 0x7F, 0x3F, 0x0F, // char 'U' (0x0055/85)
    0x20, 0xE0, 0xE0, 0xE0, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xE0, 0xE0, 0xE0, 0x20, 0x00, 0x01, 0x07, 0x3F, 0xFF, 0xFC, 0xF0, 0xC0, 0x00, 0x00, 0x00, 0xC0, 0xF0, 0xFE, 0xFF, 0x1F, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x1F, 0xFF, 0xFE, 0xF8, 0xFE, 0x7F, 0x1F, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, // char 'V' (0x0056/86)
    0x20, 0xE0, 0xE0, 0xE0, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0xE0, 0xE0, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0xE0, 0xE0, 0x60, 0x00, 0x00, 0x07, 0x3F, 0xFF, 0xFC, 0xE0, 0x00, 0x00, 0x80, 0xF0, 0xFE, 0xFF, 0x1F, 0x0F, 0x7F, 0xFF, 0xF8, 0x80, 0x00, 0x00, 0xC0, 0xF8, 0xFF, 0x7F, 0x0F, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x0F, 0x7F, 0xFF, 0xF8, 0xFF, 0x7F, 0x07, 0x00, 0x00, 0x00, 0x00, 0x03, 0x3F, 0xFF, 0xFC, 0xFE, 0xFF, 0x1F, 0x03, 0x00, 0x00, 0x00, 0x00, // char 'W' (0x0057/87)
    0x00, 0x20, 0xE0, 0xE0, 0xE0, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0xE0, 0xE0, 0xE0, 0x20, 0x00, 0x00, 0x00, 0x00, 0x01, 0x07, 0x0F, 0xBF, 0xFE, 0xF8, 0xF8, 0xFE, 0xBF, 0x0F, 0x07, 0x01, 0x00, 0x00, 0x00, 0x80, 0xC0, 0xE0, 0xF8, 0xFC, 0x3F, 0x1F, 0x07, 0x03, 0x01, 0x07, 0x0F, 0x3F, 0xFC, 0xF8, 0xE0, 0xC0, 0x80, // char 'X' (0x0058/88)
    0x20, 0xE0, 0xE0, 0xC0, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xE0, 0xE0, 0xE0, 0x20, 0x00, 0x00, 0x03, 0x07, 0x1F, 0x3E, 0xF8, 0xF0, 0xC0, 0xF0, 0xFC, 0x3E, 0x1F, 0x07, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // char 'Y' (0x0059/89)
    0x00, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xC0, 0xE0, 0xF8, 0x7C, 0x3E, 0x1F, 0x0F, 0x03, 0x01, 0x00, 0xE0, 0xF0, 0xF8, 0xFC, 0xFF, 0xEF, 0xE7, 0xE3, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, // char 'Z' (0x005A/90)
    0xF0, 0xF0, 0xF0, 0x30, 0x30, 0x30, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, // char '[' (0x005B/91)
    0x10, 0x70, 0xF0, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x07, 0x1F, 0xFC, 0xF0, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x0F, 0x3F, 0xFC, 0xF0, 0xC0, // char '\' (0x005C/92)
    0x30, 0x30, 0x30, 0x30, 0xF0, 0xF0, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, // char ']' (0x005D/93)
    0x08, 0x0C, 0x0F, 0x07, 0x03, 0x07, 0x0E, 0x0C, 0x08, // char '^' (0x005E/94)
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, // char '_' (0x005F/95)
    0x20, 0x70, 0xF0, 0xE0, 0xC0, 0x80, // char '`' (0x0060/96)
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x18, 0x1C, 0x8C, 0x8C, 0x8E, 0x8E, 0x8C, 0x9C, 0xFC, 0xF8, 0xE0, 0x3C, 0x7E, 0xFF, 0xE3, 0xE3, 0xC3, 0xC1, 0xE1, 0x61, 0x71, 0xFF, 0xFF, 0xFF, // char 'a' (0x0061/97)
    0xF0, 0xF0, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x70, 0x18, 0x1C, 0x0C, 0x0E, 0x0E, 0x1C, 0x1C, 0x7C, 0xF8, 0xF0, 0xC0, 0xFF, 0xFF, 0xFF, 0x38, 0x70, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xF0, 0x78, 0x3F, 0x1F, 0x07, // char 'b' (0x0062/98)
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xE0, 0xF8, 0x78, 0x3C, 0x1C, 0x0C, 0x0E, 0x0E, 0x1C, 0x1C, 0x3C, 0x18, 0x00, 0x07, 0x1F, 0x3F, 0x7C, 0xF0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0x70, 0x70, 0x20, // char 'c' (0x0063/99)
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0, 0xF0, 0xF0, 0x80, 0xF0, 0xF8, 0x7C, 0x3C, 0x1C, 0x0E, 0x0E, 0x0C, 0x1C, 0x1C, 0x78, 0xFF, 0xFF, 0xFF, 0x07, 0x1F, 0x3F, 0x7C, 0xF0, 0xE0, 0xE0, 0xC0, 0xE0, 0xE0, 0x70, 0x38, 0xFF, 0xFF, 0xFF, // char 'd' (0x0064/100)
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xF0, 0xF8, 0x78, 0x1C, 0x1C, 0x0C, 0x0E, 0x0C, 0x0C, 0x1C, 0x7C, 0xF8, 0xF0, 0x80, 0x07, 0x1F, 0x3F, 0x7B, 0xE3, 0xE3, 0xE3, 0xC3, 0xE3, 0xE3, 0xE3, 0x73, 0x63, 0x23, 0x03, // char 'e' (0x0065/101)
    0x00, 0x00, 0xC0, 0xE0, 0xF0, 0x70, 0x30, 0x30, 0x70, 0x20, 0x0C, 0x0C, 0xFF, 0xFF, 0xFF, 0x0C, 0x0C, 0x0C, 0x0C, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, // char 'f' (0x0066/102)
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0xF0, 0xF8, 0x7C, 0x1C, 0x0C, 0x0E, 0x0C, 0x0C, 0x1C, 0x18, 0xFC, 0xFC, 0xFC, 0x03, 0x1F, 0x3F, 0x3C, 0x78, 0x70, 0x70, 0x60, 0x70, 0x70, 0x38, 0x9F, 0xFF, 0xFF, // char 'g' (0x0067/103)
    0xF0, 0xF0, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x70, 0x18, 0x0C, 0x0C, 0x0E, 0x0C, 0x1C, 0xFC, 0xF8, 0xF0, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, // char 'h' (0x0068/104)
    0x70, 0x78, 0x70, 0x20, 0xFC, 0xFC, 0xFC, 0x00, 0xFF, 0xFF, 0xFF, 0x00, // char 'i' (0x0069/105)
    0x00, 0x00, 0x00, 0x00, 0x00, 0x70, 0x78, 0x70, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0xFC, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x00, // char 'j' (0x006A/106)
    0xF0, 0xF0, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x80, 0xC0, 0xE0, 0xF0, 0xF8, 0x3C, 0x1C, 0x0C, 0x04, 0x02, 0xFF, 0xFF, 0xFF, 0x0F, 0x07, 0x03, 0x07, 0x1F, 0x3E, 0xFC, 0xF0, 0xE0, 0x80, // char 'k' (0x006B/107)
    0xF0, 0xF0, 0xF0, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, // char 'l' (0x006C/108)
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0xFC, 0xFC, 0x70, 0x18, 0x0C, 0x0C, 0x0E, 0x0E, 0x1C, 0xFC, 0xF8, 0xF0, 0x38, 0x1C, 0x0C, 0x0C, 0x0E, 0x0C, 0x1C, 0xFC, 0xF8, 0xF0, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, // char 'm' (0x006D/109)
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0xFC, 0xFC, 0x70, 0x18, 0x0C, 0x0C, 0x0E, 0x0C, 0x1C, 0xFC, 0xF8, 0xF0, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, // char 'n' (0x006E/110)
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xE0, 0xF8, 0x78, 0x3C, 0x1C, 0x0C, 0x0E, 0x0E, 0x0C, 0x1C, 0x3C, 0xF8, 0xF0, 0xE0, 0x07, 0x1F, 0x3F, 0x7C, 0xF0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0x70, 0x7F, 0x3F, 0x0F, // char 'o' (0x006F/111)
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0xFC, 0xFC, 0x70, 0x18, 0x1C, 0x0C, 0x0E, 0x0E, 0x1C, 0x1C, 0x7C, 0xF8, 0xF0, 0xC0, 0xFF, 0xFF, 0xFF, 0x38, 0x70, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xF0, 0x78, 0x3F, 0x1F, 0x07, // char 'p' (0x0070/112)
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xF0, 0xF8, 0x7C, 0x3C, 0x1C, 0x0E, 0x0E, 0x0C, 0x1C, 0x18, 0x70, 0xFC, 0xFC, 0xFC, 0x07, 0x1F, 0x3F, 0x7C, 0xF0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0x70, 0x38, 0xFF, 0xFF, 0xFF, // char 'q' (0x0071/113)
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0xFC, 0xFC, 0xF0, 0x38, 0x1C, 0x1C, 0x1E, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, // char 'r' (0x0072/114)
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF8, 0xFC, 0xFC, 0x8C, 0x8C, 0x8E, 0x0C, 0x0C, 0x1C, 0x08, 0x60, 0x70, 0xF1, 0xE1, 0xE3, 0xE3, 0xC3, 0xE7, 0xFF, 0x7F, 0x3E, // char 's' (0x0073/115)
    0x00, 0x00, 0xE0, 0xE0, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x0C, 0xFF, 0xFF, 0xFF, 0x0C, 0x0C, 0x0C, 0x0C, 0x00, 0x00, 0x00, 0x3F, 0x7F, 0xFF, 0xE0, 0xC0, 0xE0, 0xE0, 0x40, // char 't' (0x0074/116)
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0xFC, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0xFC, 0xFC, 0x1F, 0x7F, 0x7F, 0xF0, 0xE0, 0xE0, 0xE0, 0xE0, 0x60, 0x30, 0xFF, 0xFF, 0xFF, // char 'u' (0x0075/117)
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x1C, 0x7C, 0xFC, 0xE0, 0x80, 0x00, 0x00, 0x00, 0x00, 0xC0, 0xF8, 0xFC, 0x3C, 0x0C, 0x00, 0x00, 0x00, 0x03, 0x0F, 0x7F, 0xFE, 0xF0, 0xFC, 0xFF, 0x1F, 0x07, 0x00, 0x00, 0x00, // char 'v' (0x0076/118)
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x1C, 0x7C, 0xFC, 0xF0, 0x80, 0x00, 0x00, 0x00, 0xE0, 0xFC, 0xFC, 0x7C, 0xFC, 0xF0, 0x80, 0x00, 0x00, 0x00, 0xC0, 0xF8, 0xFC, 0x3C, 0x0C, 0x00, 0x00, 0x00, 0x03, 0x1F, 0xFF, 0xFC, 0xF0, 0xFF, 0x3F, 0x07, 0x01, 0x00, 0x03, 0x0F, 0x7F, 0xFC, 0xF0, 0xFE, 0x7F, 0x0F, 0x01, 0x00, 0x00, // char 'w' (0x0077/119)
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x0C, 0x3C, 0x7C, 0xF8, 0xE0, 0xC0, 0xE0, 0xF8, 0x7C, 0x3C, 0x0C, 0x04, 0x00, 0x80, 0xE0, 0xF0, 0x7C, 0x3F, 0x1F, 0x07, 0x1F, 0x3E, 0xFC, 0xF0, 0xE0, 0x80, 0x80, // char 'x' (0x0078/120)
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x1C, 0x7C, 0xFC, 0xF0, 0x80, 0x00, 0x00, 0x00, 0x00, 0xE0, 0xF8, 0xFC, 0x3C, 0x0C, 0x00, 0x00, 0x00, 0x03, 0x0F, 0x7F, 0xFE, 0xF0, 0xFC, 0xFF, 0x1F, 0x07, 0x01, 0x00, 0x00, // char 'y' (0x0079/121)
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x0C, 0x0C, 0x0C, 0x8C, 0xCC, 0xEC, 0xFC, 0x7C, 0x3C, 0x1C, 0xE0, 0xF0, 0xFC, 0xFE, 0xFF, 0xEF, 0xE3, 0xE1, 0xE0, 0xE0, 0xE0, 0xE0, // char 'z' (0x007A/122)
    0x00, 0x00, 0xE0, 0xF0, 0xF0, 0x30, 0x30, 0xC0, 0xC0, 0xFF, 0x7F, 0x7F, 0x00, 0x00, 0x01, 0x01, 0xFF, 0xFF, 0xFE, 0x00, 0x00, // char '{' (0x007B/123)
    0xF8, 0xF8, 0xF8, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, // char '|' (0x007C/124)
    0x30, 0x30, 0xF0, 0xF0, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x7F, 0x7F, 0xFF, 0xC0, 0xC0, 0x00, 0x00, 0xFE, 0xFF, 0xFF, 0x01, 0x01, // char '}' (0x007D/125)
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x70, 0xF0, 0x38, 0x38, 0x70, 0x70, 0x60, 0xE0, 0xE0, 0xF8, 0x70, 0x30, // char '~' (0x007E/126)
    // char '' (0x007F/127)
    0x00, 0x00, 0x00, // end of unicode tables
    // FONT REQUIRES 3946 BYTES
};