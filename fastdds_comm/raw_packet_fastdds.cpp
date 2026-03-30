#include "raw_packet_fastdds.hpp"

#include <fastcdr/Cdr.h>
#include <fastcdr/FastBuffer.h>
#include <fastdds/dds/log/Log.hpp>
#include <fastdds/dds/subscriber/SampleInfo.hpp>

using eprosima::fastcdr::Cdr;
using eprosima::fastcdr::FastBuffer;
using eprosima::fastdds::dds::DataReader;
using eprosima::fastdds::dds::DomainParticipant;
using eprosima::fastdds::dds::DomainParticipantFactory;
using eprosima::fastdds::dds::PUBLISHER_QOS_DEFAULT;
using eprosima::fastdds::dds::SUBSCRIBER_QOS_DEFAULT;
using eprosima::fastdds::dds::TOPIC_QOS_DEFAULT;
using eprosima::fastrtps::rtps::SerializedPayload_t;

namespace {
constexpr const char *kTypeName = "RawPacketType";
}

RawPacketPubSubType::RawPacketPubSubType()
{
    setName(kTypeName);
    m_typeSize = static_cast<uint32_t>(4 + 1024 + 4);
    m_isGetKeyDefined = false;
}

bool RawPacketPubSubType::serialize(
    void *data,
    SerializedPayload_t *payload)
{
    const auto *packet = static_cast<const RawPacket *>(data);

    FastBuffer fastbuffer(reinterpret_cast<char *>(payload->data), payload->max_size);
    Cdr ser(fastbuffer);
    payload->encapsulation = ser.endianness() == Cdr::BIG_ENDIANNESS ? CDR_BE : CDR_LE;
    ser.serialize_encapsulation();

    ser << packet->bytes;
    payload->length = static_cast<uint32_t>(ser.getSerializedDataLength());
    return true;
}

bool RawPacketPubSubType::deserialize(SerializedPayload_t *payload, void *data)
{
    auto *packet = static_cast<RawPacket *>(data);

    FastBuffer fastbuffer(reinterpret_cast<char *>(payload->data), payload->length);
    Cdr deser(fastbuffer);
    deser.read_encapsulation();
    payload->encapsulation = deser.endianness() == Cdr::BIG_ENDIANNESS ? CDR_BE : CDR_LE;

    deser >> packet->bytes;
    return true;
}

std::function<uint32_t()> RawPacketPubSubType::getSerializedSizeProvider(void *data)
{
    return [data]() {
        const auto *packet = static_cast<const RawPacket *>(data);
        return static_cast<uint32_t>(4 + packet->bytes.size() + 4);
    };
}

void *RawPacketPubSubType::createData()
{
    return new RawPacket();
}

void RawPacketPubSubType::deleteData(void *data)
{
    delete static_cast<RawPacket *>(data);
}

bool RawPacketPubSubType::getKey(void *, eprosima::fastrtps::rtps::InstanceHandle_t *, bool)
{
    return false;
}

RawPacketPublisher::RawPacketPublisher()
    : participant_(nullptr), publisher_(nullptr), topic_(nullptr), writer_(nullptr),
      type_(new RawPacketPubSubType())
{
}

RawPacketPublisher::~RawPacketPublisher()
{
    if (participant_ != nullptr) {
        DomainParticipantFactory::get_instance()->delete_participant(participant_);
    }
}

bool RawPacketPublisher::init(const std::string &topic_name, uint32_t domain_id)
{
    auto pqos = eprosima::fastdds::dds::PARTICIPANT_QOS_DEFAULT;
    pqos.name("raw_packet_publisher");
    participant_ = DomainParticipantFactory::get_instance()->create_participant(domain_id, pqos);
    if (participant_ == nullptr) {
        return false;
    }

    type_.register_type(participant_);
    topic_ = participant_->create_topic(topic_name, type_.get_type_name(), TOPIC_QOS_DEFAULT);
    if (topic_ == nullptr) {
        return false;
    }

    publisher_ = participant_->create_publisher(PUBLISHER_QOS_DEFAULT, nullptr);
    if (publisher_ == nullptr) {
        return false;
    }

    writer_ = publisher_->create_datawriter(topic_, eprosima::fastdds::dds::DATAWRITER_QOS_DEFAULT, nullptr);
    return writer_ != nullptr;
}

bool RawPacketPublisher::publish(const std::vector<uint8_t> &bytes)
{
    if (writer_ == nullptr) {
        return false;
    }
    packet_.bytes = bytes;
    return writer_->write(&packet_) == eprosima::fastrtps::types::ReturnCode_t::RETCODE_OK;
}

RawPacketSubscriber::Listener::Listener(PacketCallback cb) : callback_(std::move(cb))
{
}

void RawPacketSubscriber::Listener::on_data_available(DataReader *reader)
{
    RawPacket packet;
    eprosima::fastdds::dds::SampleInfo info;

    while (reader->take_next_sample(&packet, &info) == eprosima::fastrtps::types::ReturnCode_t::RETCODE_OK) {
        if (info.valid_data && callback_) {
            callback_(packet.bytes);
        }
    }
}

RawPacketSubscriber::RawPacketSubscriber()
    : participant_(nullptr), subscriber_(nullptr), topic_(nullptr), reader_(nullptr),
      type_(new RawPacketPubSubType())
{
}

RawPacketSubscriber::~RawPacketSubscriber()
{
    if (participant_ != nullptr) {
        DomainParticipantFactory::get_instance()->delete_participant(participant_);
    }
}

bool RawPacketSubscriber::init(const std::string &topic_name, PacketCallback cb, uint32_t domain_id)
{
    auto pqos = eprosima::fastdds::dds::PARTICIPANT_QOS_DEFAULT;
    pqos.name("raw_packet_subscriber");
    participant_ = DomainParticipantFactory::get_instance()->create_participant(domain_id, pqos);
    if (participant_ == nullptr) {
        return false;
    }

    type_.register_type(participant_);
    topic_ = participant_->create_topic(topic_name, type_.get_type_name(), TOPIC_QOS_DEFAULT);
    if (topic_ == nullptr) {
        return false;
    }

    subscriber_ = participant_->create_subscriber(SUBSCRIBER_QOS_DEFAULT, nullptr);
    if (subscriber_ == nullptr) {
        return false;
    }

    listener_ = std::make_unique<Listener>(std::move(cb));
    reader_ = subscriber_->create_datareader(
        topic_,
        eprosima::fastdds::dds::DATAREADER_QOS_DEFAULT,
        listener_.get());

    return reader_ != nullptr;
}
