import struct


PATH_FIELD_SIZE = 32

CMD_FILE_PACKAGE = 0xF0
CMD_FILE_REQUEST = 0xF1
CMD_FILE_DATA_REQUEST = 0xF2
CMD_FILE_DATA = 0xF3
CMD_FILE_TRANSFER_DONE = 0xF4

FILE_PACKAGE_STRUCT = struct.Struct(f"<BI{PATH_FIELD_SIZE}sI")
FILE_REQUEST_STRUCT = struct.Struct(f"<B{PATH_FIELD_SIZE}s")
FILE_DATA_REQUEST_STRUCT = struct.Struct("<BI")


def encode_path_bytes(path: str) -> bytes:
    return path.encode("utf-8")[:PATH_FIELD_SIZE].ljust(PATH_FIELD_SIZE, b"\x00")


def build_file_package_payload(path: str, total_packages: int, cumulative_crc: int) -> bytes:
    return FILE_PACKAGE_STRUCT.pack(
        CMD_FILE_PACKAGE,
        total_packages,
        encode_path_bytes(path),
        cumulative_crc,
    )


def build_file_request_payload(path: str) -> bytes:
    return FILE_REQUEST_STRUCT.pack(CMD_FILE_REQUEST, encode_path_bytes(path))


def build_file_data_request_payload(seq_id: int) -> bytes:
    return FILE_DATA_REQUEST_STRUCT.pack(CMD_FILE_DATA_REQUEST, seq_id)
