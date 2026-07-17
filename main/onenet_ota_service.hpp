#ifndef ONENET_OTA_SERVICE_HPP
#define ONENET_OTA_SERVICE_HPP

#include "at_modem.h"
#include "mqtt.h"
#include "onenet_config.h"

#include <cstdint>
#include <mutex>
#include <string>

class OneNetOtaService {
public:
  OneNetOtaService(AtModem *modem, Mqtt *mqtt,
                   const onenet_config_t &config);

  const std::string &inform_topic() const { return inform_topic_; }
  bool HandleMqttMessage(const std::string &topic,
                         const std::string &payload);
  void OnOnline();
  void Service();

private:
  AtModem *modem_;
  Mqtt *mqtt_;
  onenet_config_t config_;
  std::string inform_topic_;
  std::string inform_reply_topic_;
  std::mutex mutex_;
  std::string pending_inform_id_;
  bool check_requested_ = false;
  bool version_reported_ = false;
  uint64_t next_check_epoch_ = 0U;

  bool PublishInformReply();
  bool ReportVersion();
  bool ReportPendingResult();
  bool CheckAndApply();
};

#endif
