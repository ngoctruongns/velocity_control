#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/dds/subscriber/DataReaderListener.hpp>
#include <fastdds/dds/subscriber/Subscriber.hpp>
#include <fastdds/dds/topic/Topic.hpp>
#include <fastdds/dds/topic/TopicDataType.hpp>

struct RawPacket {
    std::vector<uint8_t> bytes;
};

class RawPacketPubSubType : public eprosima::fastdds::dds::TopicDataType {
public:
    RawPacketPubSubType();

    bool serialize(
        void *data,
        eprosima::fastrtps::rtps::SerializedPayload_t *payload) override;

    bool deserialize(
        eprosima::fastrtps::rtps::SerializedPayload_t *payload,
        void *data) override;

    std::function<uint32_t()> getSerializedSizeProvider(void *data) override;

    void *createData() override;

    void deleteData(void *data) override;

    bool getKey(
        void *data,
        eprosima::fastrtps::rtps::InstanceHandle_t *handle,
        bool force_md5 = false) override;
};

class RawPacketPublisher {
public:
    RawPacketPublisher();
    ~RawPacketPublisher();

    RawPacketPublisher(const RawPacketPublisher &) = delete;
    RawPacketPublisher &operator=(const RawPacketPublisher &) = delete;

    bool init(const std::string &topic_name, uint32_t domain_id = 0);
    bool publish(const std::vector<uint8_t> &bytes);

private:
    eprosima::fastdds::dds::DomainParticipant *participant_;
    eprosima::fastdds::dds::Publisher *publisher_;
    eprosima::fastdds::dds::Topic *topic_;
    eprosima::fastdds::dds::DataWriter *writer_;
    eprosima::fastdds::dds::TypeSupport type_;
    RawPacket packet_;
};

class RawPacketSubscriber {
public:
    using PacketCallback = std::function<void(const std::vector<uint8_t> &)>;

    RawPacketSubscriber();
    ~RawPacketSubscriber();

    RawPacketSubscriber(const RawPacketSubscriber &) = delete;
    RawPacketSubscriber &operator=(const RawPacketSubscriber &) = delete;

    bool init(const std::string &topic_name, PacketCallback cb, uint32_t domain_id = 0);

private:
    class Listener : public eprosima::fastdds::dds::DataReaderListener {
    public:
        explicit Listener(PacketCallback cb);
        void on_data_available(eprosima::fastdds::dds::DataReader *reader) override;

    private:
        PacketCallback callback_;
    };

    eprosima::fastdds::dds::DomainParticipant *participant_;
    eprosima::fastdds::dds::Subscriber *subscriber_;
    eprosima::fastdds::dds::Topic *topic_;
    eprosima::fastdds::dds::DataReader *reader_;
    eprosima::fastdds::dds::TypeSupport type_;
    std::unique_ptr<Listener> listener_;
};
