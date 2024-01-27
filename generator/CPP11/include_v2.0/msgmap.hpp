
#pragma once

#include <algorithm>
#ifdef FREEBSD
#include <sys/endian.h>
#elif __APPLE__
#include <machine/endian.h>
#elif __NONE__
#include <sys/endian.h>
#else
#include <endian.h>
#endif
#include <type_traits>

namespace mavlink {

/**
 * Serialization helper wrapper for mavlink_message_t
 */
class MsgMap {
public:

	explicit MsgMap(mavlink_message_t *p) :
		msg(p), cmsg(p), pos(0)
	{ }

	explicit MsgMap(mavlink_message_t &p) :
		msg(&p), cmsg(&p), pos(0)
	{ }

	explicit MsgMap(const mavlink_message_t *p) :
		msg(nullptr), cmsg(p), pos(0)
	{ }

	inline void reset()
	{
		pos = 0;
	}

	inline void reset(uint32_t msgid, uint8_t len)
	{
		assert(msg);

		msg->msgid = msgid;	// necessary for finalize
		msg->len = len;		// needed only for deserialization w/o finalize
		pos = 0;
	}

	template<typename T>
	void operator<< (const T data);

	template<class T, size_t Size>
	void operator<< (const std::array<T, Size> &data);

	template<typename T>
	void operator>> (T &data);

	template<class T, size_t Size>
	void operator>> (std::array<T, Size> &data);

private:
	mavlink_message_t *msg;		// for serialization
	const mavlink_message_t *cmsg;	// for deserialization
	size_t pos;
};

namespace impl {

template<size_t N>
struct UintBufferHelper;

template<>
struct UintBufferHelper<1>
{
    typedef uint8_t Type;
};

template<>
struct UintBufferHelper<2>
{
    typedef uint16_t Type;
};

template<>
struct UintBufferHelper<4>
{
    typedef uint32_t Type;
};

template<>
struct UintBufferHelper<8>
{
    typedef uint64_t Type;
};

template<typename T>
struct UintBuffer
{
    typedef typename UintBufferHelper<sizeof(T)>::Type Type;
};


template<typename T>
T to_little_endian_internal(T);

template<>
inline uint8_t to_little_endian_internal<uint8_t>(uint8_t data)
{
    return data;
}

template<>
inline uint16_t to_little_endian_internal<uint16_t>(uint16_t data)
{
    return htole16(data);
}

template<>
inline uint32_t to_little_endian_internal<uint32_t>(uint32_t data)
{
    return htole32(data);
}

template<>
inline uint64_t to_little_endian_internal<uint64_t>(uint64_t data)
{
    return htole64(data);
}

template<typename T>
typename std::enable_if<std::is_floating_point<T>::value, typename UintBuffer<T>::Type>::type to_little_endian(T data)
{
    typedef typename UintBuffer<T>::Type ReturnType;
    ReturnType buf;
    memcpy(&buf, &data, sizeof(ReturnType));
    return to_little_endian_internal<ReturnType>(buf);
}

template<typename T>
typename std::enable_if<std::is_integral<T>::value, typename UintBuffer<T>::Type>::type to_little_endian(T data)
{
    return to_little_endian_internal<typename UintBuffer<T>::Type>(data);
}

template<typename T>
T to_host_from_little_endian_internal(T);

template<>
inline uint8_t to_host_from_little_endian_internal<uint8_t>(uint8_t data)
{
    return data;
}

template<>
inline uint16_t to_host_from_little_endian_internal<uint16_t>(uint16_t data)
{
    return le16toh(data);
}

template<>
inline uint32_t to_host_from_little_endian_internal<uint32_t>(uint32_t data)
{
    return le32toh(data);
}

template<>
inline uint64_t to_host_from_little_endian_internal<uint64_t>(uint64_t data)
{
    return le64toh(data);
}

template<typename TOutput, typename TInput,
         class = typename std::enable_if<std::is_unsigned<TInput>::value>::type>
typename std::enable_if<std::is_floating_point<TOutput>::value, TOutput>::type to_host_from_little_endian(TInput data)
{
    static_assert(sizeof(TInput) == sizeof(TOutput), "Size of input and output must match");
    data = to_host_from_little_endian_internal(data);

    TOutput buf;
    memcpy(&buf, &data, sizeof(TOutput));
    return buf;
}

template<typename TOutput, typename TInput,
         class = typename std::enable_if<std::is_unsigned<TInput>::value>::type>
typename std::enable_if<std::is_integral<TOutput>::value, TOutput>::type to_host_from_little_endian(TInput data)
{
    static_assert(sizeof(TInput) == sizeof(TOutput), "Size of input and output must match");
    return to_host_from_little_endian_internal(data);
}

} // namespace impl
} // namespace mavlink

// implementation

template<typename T>
void mavlink::MsgMap::operator<< (const T data)
{
    assert(msg);
    assert(pos + sizeof(T) <= MAVLINK_MAX_PAYLOAD_LEN);

    auto data_le = mavlink::impl::to_little_endian<T>(data);
    memcpy(&_MAV_PAYLOAD_NON_CONST(msg)[pos], &data_le, sizeof(data_le));
    pos += sizeof(T);
}

template<class T, size_t Size>
void mavlink::MsgMap::operator<< (const std::array<T, Size> &data)
{
	for (auto &v : data) {
		*this << v;
	}
}

template<typename T>
void mavlink::MsgMap::operator>> (T &data)
{
    assert(cmsg);
    assert(pos + sizeof(T) <= MAVLINK_MAX_PAYLOAD_LEN);

    ssize_t remaining_non_zero_data = cmsg->len - pos;
    typename mavlink::impl::UintBuffer<T>::Type buf;

    if (static_cast<ssize_t>(sizeof(T)) <= remaining_non_zero_data) { // field is not truncated
        memcpy(&buf, &_MAV_PAYLOAD(cmsg)[pos], sizeof(T));
    } else if (remaining_non_zero_data <= 0) {
        // there is no non-zero data left, so just fill with 0
        buf = 0;
    } else { // field is trimmed - pad with zeroes
        // here remaining_non_zero_data < sizeof(T) holds
        size_t non_zero_count = std::max<decltype(remaining_non_zero_data)>(remaining_non_zero_data, 0);
        size_t pad_zero_count = sizeof(T) - non_zero_count;

        std::array<char, sizeof(T)> raw_buf;
        memcpy(raw_buf.data(), &_MAV_PAYLOAD(cmsg)[pos], non_zero_count);
        memset(raw_buf.data() + non_zero_count, 0, pad_zero_count);

        memcpy(&buf, raw_buf.data(), raw_buf.size());
    }

    data = mavlink::impl::to_host_from_little_endian<T>(buf);
    pos += sizeof(T);
}

template<class T, size_t Size>
void mavlink::MsgMap::operator>> (std::array<T, Size> &data)
{
	for (auto &v : data) {
		*this >> v;
	}
}
