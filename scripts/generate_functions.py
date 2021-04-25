from csnake import CodeWriter, Variable, Function
import json
import collections
from pathlib import Path

SYNC_BYTE = '0xAA'
PROTO_BYTE0 = '0x40'
PROTO_BYTE1 = '0x9B'

CRC8_LUT = [
	0xb2, 0x85, 0xdc, 0xeb, 0x6e, 0x59, 0x00, 0x37, 0x3d, 0x0a, 0x53, 0x64,
	0xe1, 0xd6, 0x8f, 0xb8, 0x9b, 0xac, 0xf5, 0xc2, 0x47, 0x70, 0x29, 0x1e,
	0x14, 0x23, 0x7a, 0x4d, 0xc8, 0xff, 0xa6, 0x91, 0xe0, 0xd7, 0x8e, 0xb9,
	0x3c, 0x0b, 0x52, 0x65, 0x6f, 0x58, 0x01, 0x36, 0xb3, 0x84, 0xdd, 0xea,
	0xc9, 0xfe, 0xa7, 0x90, 0x15, 0x22, 0x7b, 0x4c, 0x46, 0x71, 0x28, 0x1f,
	0x9a, 0xad, 0xf4, 0xc3, 0x16, 0x21, 0x78, 0x4f, 0xca, 0xfd, 0xa4, 0x93,
	0x99, 0xae, 0xf7, 0xc0, 0x45, 0x72, 0x2b, 0x1c, 0x3f, 0x08, 0x51, 0x66,
	0xe3, 0xd4, 0x8d, 0xba, 0xb0, 0x87, 0xde, 0xe9, 0x6c, 0x5b, 0x02, 0x35,
	0x44, 0x73, 0x2a, 0x1d, 0x98, 0xaf, 0xf6, 0xc1, 0xcb, 0xfc, 0xa5, 0x92,
	0x17, 0x20, 0x79, 0x4e, 0x6d, 0x5a, 0x03, 0x34, 0xb1, 0x86, 0xdf, 0xe8,
	0xe2, 0xd5, 0x8c, 0xbb, 0x3e, 0x09, 0x50, 0x67]

def add_funk(funk):
	cw.add_function_definition(funk)
	cwh.add_function_prototype(funk)


# Recursively flatten the JSON file
def thingify(l, f_str):
	for x in l:
		if isinstance(x, dict):
			if "members" in x:
				thingify(x["members"], f_str + x["name"] + "_") 
			elif "inputs" in x:
				thingify(x["inputs"],"NOT_SUPPORTED_")
			else:
				if x["type"] == 'uint32':
					s = 4
					if x['access'].find('w') != 1:
						arg1 = Variable(x['name'], 'uint32_t')
						fun = Function(f_str + "w_" + x['name'], 'void') 
						fun.add_code('buff[0] = ' + SYNC_BYTE + ';')
						fun.add_code('buff[1] = ' + str(s + 8) + ';')
						fun.add_code('buff[2] = ' + str(hex(CRC8_LUT[s + 8])) + ';')
						fun.add_argument(arg1)
						add_funk(fun)
					if x['access'].find('r') != 1:
						fun = Function(f_str + "r_" + x['name'], "uin32_t")
						fun.add_code('buff[0] = ' + SYNC_BYTE + ';')
						fun.add_code('buff[1] = ' + str(8) + ';')
						fun.add_code('buff[2] = ' + str(hex(CRC8_LUT[8])) + ';')
						add_funk(fun)


cw = CodeWriter()
cwh = CodeWriter()
cw.include('"odrive_uart_calls.h"')

fun = Function('crc16', 'void')
fun.add_argument(Variable('payload[]', 'uint8_t'))
fun.add_argument(Variable('payload_size', 'uint8_t'))
fun.add_argument(Variable('crc[2]', 'uint8_t'))
fun.add_code('uint16_t result = CRC16_INIT;')
fun.add_code('for (size_t i = 0; i < payload_size; i++) {')
fun.add_code('result ^= payload[i] << 8;')
fun.add_code('for (size_t j = 8; j > 0; j--) {')
fun.add_code('if (result & 0b1000000000000000) {')
fun.add_code('result = (result << 1) ^ CRC16_POLY;')
fun.add_code('} else {')
fun.add_code('result = result << 1;')
fun.add_code('}')
fun.add_code('}')
fun.add_code('}')
fun.add_code('crc[0] = result & 0x00FF;')
fun.add_code('crc[1] = (result >> 8) & 0x00FF;')
fun.add_code('}')
add_funk(fun)

home = str(Path.home())
with open(home + '/.cache/odrivetool/fibre_schema_cache_2604684226') as f:
	data = json.load(f)
thingify(data, "odrive_")

cw.write_to_file(home + '/Documents/esp-ws/diff-drive-esp32-uros/components/odrive/odrive_uart_calls.c')
cwh.write_to_file(home + '/Documents/esp-ws/diff-drive-esp32-uros/components/odrive/odrive_uart_calls.h')


