from std_msgs.msg import ByteMultiArray

def str_to_bytearray(msg: str) -> ByteMultiArray:
    """Converts an ascii string to a ByteMultiArray (each element of the array storring a char of the string)

    Args:
        msg (str): ascii string to convert

    Returns:
        ByteMultiArray: converted array
        
    Raises:
        UnicodeEncodeError when passed string is not ascii
    """
    array = ByteMultiArray()
    array.data = [char.to_bytes(1, "big") for char in msg.encode("ascii", errors="strict")]
    return array