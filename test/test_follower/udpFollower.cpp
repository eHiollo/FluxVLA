#include "kaanh/follower/udpFollower.hpp"
#include "kaanh/follower/Filter.hpp"
#include "kaanh/middleware/middleware.hpp"
#include "aris/core/socket_multi_io.hpp"
#include "kaanh/multi_motion/motion_config.hpp"
#include "kaanh/package/sensor_filter.hpp"
#include "kaanh/package/force_algorithm.hpp"
#include "kaanh/general/json.hpp"
#include "kaanh/multi_motion/motion_plan.hpp"
#include "kaanh/multi_motion/motion_manager.hpp"
#include "kaanh/multi_motion/jog_plan.hpp"
#include "kaanhcode/pgm_calculator.hpp"
#include "kaanh/translate/translate.hpp"
static bool flag = false;
namespace kaanh::follower{
    using aris::core::Command;
    using namespace kaanh::package;
    using namespace kaanh::multi_motion;

    struct FollowerParamBase::Imp{

        // 全局单例
        kaanh::robot::Robot *robot{nullptr};
        kaanh::multi_motion::MoveParam* mvp{nullptr};
        aris::dynamic::MultiModel* multi_model{nullptr};
        kaanh::multi_motion::TrajectoryGeneratorManager* tgm{nullptr};
        // kaanh::multi_motion::SingularProcessorManager* spm{nullptr};
        kaanh::multi_motion::ToolWobjManager* twm{nullptr};
        kaanh::multi_motion::EeManager* eem{nullptr};
        kaanh::log::Sqlite3Log* log{nullptr};

        std::unique_ptr<aris::core::SocketServer> server_{new aris::core::SocketServer};
        aris::plan::TrajectoryGenerator* tg_ = nullptr;
        int tg_id_;
        std::recursive_mutex mutex_;
        std::atomic<std::uint64_t> last_id = 0;

    };
    auto FollowerParamBase::multiModel()-> aris::dynamic::MultiModel*{ return imp_->multi_model;}
    auto FollowerParamBase::robot()-> kaanh::robot::Robot*{return imp_->robot;}
    auto FollowerParamBase::moveParam()-> kaanh::multi_motion::MoveParam*{return imp_->mvp;}
    auto FollowerParamBase::twManager()->kaanh::multi_motion::ToolWobjManager*{ return imp_->twm;}
    auto FollowerParamBase::eeManager()-> kaanh::multi_motion::EeManager* { return imp_->eem;}
    auto FollowerParamBase::tgManager()-> kaanh::multi_motion::TrajectoryGeneratorManager*{ return imp_->tgm;}; 
    // auto FollowerParamBase::spManager()-> kaanh::multi_motion::SingularProcessorManager*{ return imp_->spm;}; 
    auto FollowerParamBase::log()-> kaanh::log::Sqlite3Log*{return imp_->log;}
    auto FollowerParamBase::socketServer()->aris::core::SocketServer& { return *imp_->server_; }
    auto FollowerParamBase::setSocketServer(aris::core::SocketServer *server)->void { imp_->server_.reset(server); }
    auto FollowerParamBase::tg()-> aris::plan::TrajectoryGenerator* {return imp_->tg_;}
    auto FollowerParamBase::tgId()->int  {return imp_->tg_id_;}
    auto FollowerParamBase::setTgId(const int tg_id) ->void { imp_->tg_id_ = tg_id;}
    auto FollowerParamBase::setLastId(std::uint64_t id)->void { imp_->last_id.store(id);}
    auto FollowerParamBase::lastId()->std::uint64_t { return imp_->last_id.load();}
    auto FollowerParamBase::getInstance(bool is_cart)->void
    {
        auto& cs = aris::server::ControlServer::instance();
        imp_->multi_model = dynamic_cast<aris::dynamic::MultiModel *>(&cs.model()); //模型
        imp_->robot = &dynamic_cast<kaanh::robot::Robot &>(cs.customModule());
        imp_->tgm = &TrajectoryGeneratorManager::instance(); //规划器管理
        // imp_->spm = &SingularProcessorManager::instance(); //sp管理
        imp_->twm = &ToolWobjManager::instance();            // 工具工件管理
        imp_->eem = &EeManager::instance();
        imp_->mvp = &kaanh::multi_motion::MoveParam::instance();
        imp_->log = &kaanh::log::Sqlite3Log::instance();

        imp_->tg_ = &kaanh::multi_motion::TrajectoryGeneratorManager::instance().getTrajectoryGenerator(tgId(),is_cart);
    }
    FollowerParamBase::FollowerParamBase() : imp_(new Imp) {}
    FollowerParamBase::~FollowerParamBase() = default;

    struct FollowerAjParam::Imp{

        std::recursive_mutex mutex_;

        aris::core::CommandParser parser_;
        // plan
        std::vector<aris::dynamic::Motion*> ctrl_motions{nullptr};
        std::vector<aris::dynamic::EEType> ctrl_motion_types;
        std::vector<aris::Size> ctrl_motion_id;
        std::vector<double> vels;
        std::vector<double> accs;
        std::vector<double> jerks;
        std::vector<double> zones;
        std::vector<double> last_pos;
        std::uint64_t cmd_id = 0;

        // 不规划直接写入电机 
        std::vector<kaanh::package::LowPass> lp_vec;
        std::vector<MeanFilter> mf_vec;
        std::vector<kaanh::package::SensorFilter> filter_vec;
        std::unique_ptr<aris::core::LockFreeArrayQueue<std::vector<double>>> aj_data_queue_ { new aris::core::LockFreeArrayQueue<std::vector<double>>};

    };
    auto FollowerAjParam::init()->void
    {

        {
            Command cmd;
            aris::core::fromXmlString(cmd,
                "<Command name=\"follower\">"
                "   <GroupParam>"
                "       <Param name=\"data\"/>"
                "   </GroupParam>"
                "</Command>");
            imp_->parser_.commandPool().push_back(cmd);
        }

        imp_->parser_.init();


        socketServer().setOnReceivedMsg([&](aris::core::SocketServer* server, aris::core::SOCKET_T sock, aris::core::Msg& msg)->int {
           ARIS_COUT << "server recv " + std::to_string(msg.size()) + " bytes msg " + "from " + msg.remoteIpStr() + ":" + std::to_string(msg.remotePort()) + " -- " + msg.toString() << std::endl;
            
            // 
            if (msg.toString().find("jp") != std::string::npos)
            {
                // std::string_view cmd_name;
                // std::map<std::string_view, std::string_view> params;

                // try {
                //     std::tie(cmd_name, params) = imp_->parser_.parse(msg.toString());
                // } catch (const std::exception &err) {
                //     std::cout << err.what() << std::endl;
                // }

                try 
                {

                    auto str = msg.toString();
                    std::vector<double> pos,vel,filter_pos;
                    std::string s = str.substr(str.find("["),str.size() - str.find("[")) ;
                    auto j = nlohmann::json::parse(s);

                    pos = j.get<std::vector<double>>();
    
                    filter_pos.resize(pos.size(),0.0);

                    if( pos.size() != imp_->ctrl_motion_id.size())
                    {
                        std::cout << "pos or vel size is not equal to motor size" << std::endl;
                        aris::core::Msg s;
                        s.copy(std::string("pos or vel size is not equal to motor size"));
                        // std::cout<<"msg send: "<<j.dump()<<std::endl;
                        server->sendMsg(sock,s);
                        return 0;
                    }
                    
                    for (size_t i = 0; i < imp_->ctrl_motion_types.size(); i++)
                    {
                        if (imp_->ctrl_motion_types[i] == aris::dynamic::EEType::A)
                            pos[i] *= aris::PI / 180.0;
                        else
                            pos[i] /= 1000.0;

                        // imp_->lp_vec[i].get_filter_data(2,10,0.03,pos[i],filter_pos[i]);
                        filter_pos[i] = imp_->mf_vec[i].update(pos[i]);
                        //  filter_pos[i] = imp_->filter_vec[i].getFilteredSignalPoint(pos[i]);
                    }
                    
                    
                    aris::dynamic::dsp(1,filter_pos.size(),filter_pos.data());
                    // aris::dynamic::dsp(1,vel.size(),vel.data());

                    

                    this->addAjDataToTg(pos);
                    this->addAjDataToQueue(filter_pos);

                    return 0;
                    
                
                } catch (const std::exception &err) {
                    std::cout << err.what() << std::endl;
                }

            }
            else
            {
                std::vector<double> pos,vel,filter_pos;
                try {
                    auto j = nlohmann::json::parse(msg.toString());

                    pos =j.at("motor_pos").get<std::vector<double>>();

                    // pos = j.get<std::vector<double>>();
                    //vel =j.at("motor_vel").get<std::vector<double>>();
                    filter_pos.resize(pos.size(),0.0);
                    //auto id =j.at("id").get<std::uint64_t>();
                    if( pos.size() != imp_->ctrl_motion_id.size())
                    {
                        std::cout << "pos or vel size is not equal to motor size" << std::endl;
                        aris::core::Msg s;
                        s.copy(std::string("pos or vel size is not equal to motor size"));
                        // std::cout<<"msg send: "<<j.dump()<<std::endl;
                        server->sendMsg(sock,s);
                        return 0;
                    }
                    
                    // std::cout << "recv--------------id:" << id << "     last_id:" << imp_->last_id.load() << std::endl;
                    // if(lastId() > id)
                    //     return 0;
                    // else
                    //     setLastId(id);
                    // aris::dynamic::s_nv(pos.size(),1000,pos.data());

                    for (size_t i = 0; i < imp_->ctrl_motion_types.size(); i++)
                    {
                        if (imp_->ctrl_motion_types[i] == aris::dynamic::EEType::A)
                            pos[i] *= aris::PI / 180.0;
                        else
                            pos[i] /= 1000.0;

                        // imp_->lp_vec[i].get_filter_data(2,10,0.03,pos[i],filter_pos[i]);
                        filter_pos[i] = imp_->mf_vec[i].update(pos[i]);
                        //  filter_pos[i] = imp_->filter_vec[i].getFilteredSignalPoint(pos[i]);
                    }
                    
                    
                    aris::dynamic::dsp(1,filter_pos.size(),filter_pos.data());
                    // aris::dynamic::dsp(1,vel.size(),vel.data());

                    

                    this->addAjDataToTg(pos);
                    this->addAjDataToQueue(filter_pos);

                } catch (std::exception& e) {
                    std::cout << "recv error:"<< e.what() << std::endl;
                }


                return 0;
            }
            return 0;
        });

        socketServer().setOnReceivedConnection([](aris::core::SocketServer* server, aris::core::SOCKET_T sock, const char* remote_ip, int port)->int {
		    std::cout << " receive udp connection" << std::endl;
		    return 0;
		});
        socketServer().setOnLoseConnection([](aris::core::SocketServer* server, aris::core::SOCKET_T sock)->int {
            std::cout  << "lose udp connection" << std::endl;
            return 0;
        });

        socketServer().startServer();

        getInstance(false);

        auto& cs  = aris::server::ControlServer::instance();
        auto model = dynamic_cast<aris::dynamic::MultiModel*>(&aris::server::ControlServer::instance().model());
        auto& tgm = kaanh::multi_motion::TrajectoryGeneratorManager::instance();
        auto& mvp = kaanh::multi_motion::MoveParam::instance();

        imp_->ctrl_motions = tgm.getControlMotions(tgId());
        imp_->ctrl_motion_types = tgm.getControlMotionTypes(tgId());
        imp_->ctrl_motion_id = tgm.getControlId(tgId());

        tg()->setEeTypes(imp_->ctrl_motion_types);
        tg()->setDt(cs.master().samplePeriodNs()/1000000000.0);
    
        // init tg
        imp_->lp_vec.resize(imp_->ctrl_motions.size());
        imp_->mf_vec.resize(imp_->ctrl_motions.size(),MeanFilter(10));
        imp_->filter_vec.resize(imp_->ctrl_motions.size(),SensorFilter(SensorFilter::FilterModeType::MEANFILTER,SensorFilter::WindowSizeType::SMALLWINDOW));
        // 获取要控制的末端的参数 pos vel acc jerk zone
        for (size_t i = 0; i < imp_->ctrl_motion_id.size(); i++)
        {
            double vel_per = 10;
            vel_per /= 100.0;
            imp_->vels.push_back(0.999999 * cs.controller().motorPool()[imp_->ctrl_motion_id[i]].maxVel() * vel_per);
            imp_->accs.push_back(0.999999 * cs.controller().motorPool()[imp_->ctrl_motion_id[i]].maxAcc() * std::sqrt(vel_per));
            imp_->jerks.push_back(0.999999 * cs.controller().motorPool()[imp_->ctrl_motion_id[i]].maxAcc() * mvp.getAccDecJerkLimit().joint_jerk * std::pow(vel_per,1/3));
            if (imp_->ctrl_motion_types[i] == aris::dynamic::EEType::A)
                imp_->zones.push_back(10);
            else if (imp_->ctrl_motion_types[i] == aris::dynamic::EEType::X)
                imp_->zones.push_back(10);
        }
    }
    auto FollowerAjParam::ctrlMotions()-> const std::vector<aris::dynamic::Motion*>{ return imp_->ctrl_motions;}
    auto FollowerAjParam::ctrlMotionId()-> const std::vector<aris::Size>{ return imp_->ctrl_motion_id;}
    auto FollowerAjParam::addAjDataToTg(const std::vector<double>& pos,const std::vector<double>& vel)->void
    {
        std::unique_lock<std::recursive_mutex> lck(imp_->mutex_);
        if(vel.size())
           tg()->insertLinePos(++imp_->cmd_id, pos.data(), vel.data(), imp_->accs.data(), imp_->jerks.data(), imp_->zones.data());
        else
           tg()->insertLinePos(++imp_->cmd_id, pos.data(), imp_->vels.data(), imp_->accs.data(), imp_->jerks.data(), imp_->zones.data());
    }
    auto FollowerAjParam::clearAjDataQueue()->void { imp_->aj_data_queue_->clear();}
    auto FollowerAjParam::addAjDataToQueue(const std::vector<double>& pos)->void
    {
        imp_->aj_data_queue_->push(pos);
    }
    auto FollowerAjParam::ajDataToQueue()->const aris::core::LockFreeArrayQueue<std::vector<double>>&
    {
        return *imp_->aj_data_queue_;
    }
    auto FollowerAjParam::getAjDataFromQueue(std::vector<double>& cur_pos)->bool
    {
        if (imp_->aj_data_queue_->isEmpty())
        {
            return false;
        }
        else
        {
            return imp_->aj_data_queue_->pop(cur_pos);
        }
    }
    FollowerAjParam::FollowerAjParam() : imp_(new Imp) {}
    FollowerAjParam::~FollowerAjParam() = default;

    struct FollowerCartParam::Imp{

        std::recursive_mutex mutex_;

        // plan
        std::vector<aris::dynamic::MotionBase*> ctrl_ees{nullptr};
        std::vector<aris::dynamic::EEType> ctrl_ee_types;
        std::vector<aris::Size> ctrl_ee_id;

        // 绝对位置时使用 规划器算法//
        // 工具工件
        std::vector<aris::dynamic::Marker *> tool,hd_wobj,tool0;
        std::vector<aris::dynamic::Marker *> wobj,ex_tool,wobj0;
        std::vector<bool> is_ext_tool;
        std::vector<double> vels;
        std::vector<double> accs;
        std::vector<double> jerks;
        std::vector<double> zones;
        std::uint64_t cmd_id = 1;
        // 工具工件 外部工具手持工件
        std::vector<Size> tool_id,wobj_id;
        
        
        std::unique_ptr<aris::core::LockFreeArrayQueue<std::vector<std::vector<double>>>> cart_data_queue_ { new aris::core::LockFreeArrayQueue<std::vector<std::vector<double>>>};
        // 不规划直接写入电机 
        std::vector<kaanh::package::LowPass> lp_vec;
        std::vector<MeanFilter> mf_vec;
        std::vector<kaanh::package::SensorFilter> filter_vec;
    };
    auto FollowerCartParam::ctrlEes()-> const std::vector<aris::dynamic::MotionBase*>{ return imp_->ctrl_ees;}
    auto FollowerCartParam::ctrlEeId()-> const std::vector<aris::Size>{ return imp_->ctrl_ee_id;}
    auto FollowerCartParam::ctrlEeTypes()-> const std::vector<aris::dynamic::EEType> { return imp_->ctrl_ee_types;}
    auto FollowerCartParam::addCartDataToTg(const std::vector<double>& pos,const std::vector<double>& vel)->void
    {
        std::unique_lock<std::recursive_mutex> lck(imp_->mutex_);
        if(vel.size())
           tg()->insertLinePos(++imp_->cmd_id, pos.data(), vel.data(), imp_->accs.data(), imp_->jerks.data(), imp_->zones.data());
        else
           tg()->insertLinePos(++imp_->cmd_id, pos.data(), imp_->vels.data(), imp_->accs.data(), imp_->jerks.data(), imp_->zones.data());
    }
    auto FollowerCartParam::addDataToQueue(const std::vector<std::vector<double>>& pe)->void { imp_->cart_data_queue_->push(pe);}
    auto FollowerCartParam::getDataFromQueue(std::vector<std::vector<double>>& cur_pe)->bool 
    {
        if (imp_->cart_data_queue_->isEmpty())
        {
            return false;
        }
        else
        {
            return imp_->cart_data_queue_->pop(cur_pe);
        }
    }
    auto FollowerCartParam::clearDataQueue()->void { imp_->cart_data_queue_->clear();}
    auto FollowerCartParam::init()->void
    {
        socketServer().setOnReceivedMsg([&](aris::core::SocketServer* server, aris::core::SOCKET_T sock, aris::core::Msg& msg)->int {
        //    ARIS_COUT << "server recv " + std::to_string(msg.size()) + " bytes msg " + "from " + msg.remoteIpStr() + ":" + std::to_string(msg.remotePort()) + " -- " + msg.toString() << std::endl;
            

            try {
                std::vector<std::vector<double>> target_pe,fliter_pe;
                auto j = nlohmann::json::parse(msg.toString());
                auto type = j.at("type").get<std::string>();
                if (type == "pq")
                {
                    auto pq =j.at("pq").get<std::vector<std::vector<double>>>();
                    target_pe.resize(pq.size(),std::vector<double>(6));
                    fliter_pe.resize(pq.size(),std::vector<double>(6));
                    for (size_t i = 0; i < pq.size(); i++)
                    {
                        aris::dynamic::s_pq2pe(pq[i].data(),target_pe[i].data(),"321");
                    }
                }
                else if (type == "123")
                {
                    auto pe =j.at("pe").get<std::vector<std::vector<double>>>();
                    double pq[7]{0};
                    target_pe.resize(pe.size(),std::vector<double>(6));
                    fliter_pe.resize(pe.size(),std::vector<double>(6));
                    
                    for (size_t i = 0; i < pe.size(); i++)
                    {
                        aris::dynamic::s_pe2pq(pe[i].data(),pq,"123");
                        aris::dynamic::s_pq2pe(pq,target_pe[i].data(),"321");
                    }
                }
                else if (type == "313")
                {
                    auto pe =j.at("pe").get<std::vector<std::vector<double>>>();
                    double pq[7]{0};
                    target_pe.resize(pe.size(),std::vector<double>(6));
                    fliter_pe.resize(pe.size(),std::vector<double>(6));
                    
                    for (size_t i = 0; i < pe.size(); i++)
                    {
                        aris::dynamic::s_pe2pq(pe[i].data(),pq,"313");
                        aris::dynamic::s_pq2pe(pq,target_pe[i].data(),"321");
                    }
                }
                else // 321
                {
                    target_pe =j.at("pe").get<std::vector<std::vector<double>>>();
                    fliter_pe.resize(target_pe.size(),std::vector<double>(6));
                }
                
            

                if(target_pe.size() != imp_->ctrl_ee_id.size())
                {
                    std::cout << "The data dimension does not match the configuration" << std::endl;
                    aris::core::Msg s;
                    s.copy(std::string("The data dimension does not match the configuration"));
                    // std::cout<<"msg send: "<<j.dump()<<std::endl;
                    server->sendMsg(sock,s);
                    return 0;
                }

                for (size_t i = 0; i < target_pe.size(); i++)
                {
                    for (size_t j = 0; j < 6; j++)
                    {
                        imp_->lp_vec[6*i+j].get_filter_data(2,10,0.03,target_pe[i][j],fliter_pe[i][j]);

                        //fliter_pe[i][j] = imp_->mf_vec[6*i+j].update(pe[i][j]);
                    }
                    aris::dynamic::dsp(1,fliter_pe[i].size(),fliter_pe[i].data());
                }
                   
                // aris::dynamic::s_nv(pos.size(),1000,pos.data());

                this->addDataToQueue(fliter_pe);

            } catch (std::exception& e) {
                std::cout << "recv error:"<< e.what() << std::endl;
            }


            return 0;
		});

        socketServer().setOnReceivedConnection([](aris::core::SocketServer* server, aris::core::SOCKET_T sock, const char* remote_ip, int port)->int {
		    std::cout << " receive udp connection" << std::endl;
		    return 0;
		});
        socketServer().setOnLoseConnection([](aris::core::SocketServer* server, aris::core::SOCKET_T sock)->int {
            std::cout  << "lose udp connection" << std::endl;
            return 0;
        });

        socketServer().startServer();

        getInstance(true);

        // 初始化笛卡尔空间
        auto& cs  = aris::server::ControlServer::instance();
        auto model = dynamic_cast<aris::dynamic::MultiModel*>(&aris::server::ControlServer::instance().model());
        auto& tgm = kaanh::multi_motion::TrajectoryGeneratorManager::instance();
        auto& mvp = kaanh::multi_motion::MoveParam::instance();

        imp_->ctrl_ee_types = tgm.getControlEeTypes(tgId());
        imp_->ctrl_ees = tgm.getControlEes(tgId());
        imp_->ctrl_ee_id = tgm.getControlId(tgId());


        imp_->lp_vec.resize(6*imp_->ctrl_ees.size());
        imp_->mf_vec.resize(6*imp_->ctrl_ees.size(),MeanFilter(10));
        imp_->filter_vec.resize(6*imp_->ctrl_ees.size(),SensorFilter(SensorFilter::FilterModeType::MEANFILTER,SensorFilter::WindowSizeType::SMALLWINDOW));
        
        // // 绝对位置时 速度 转弯区
        // auto vel = ClassGeneratorAndParser::Speed::parseC(cal.eval(std::string("")));
        // auto zone = ClassGeneratorAndParser::Zone::parseC(cal.eval(std::string("")));        
        // getSpeed(vel,imp_->vels,imp_->accs,imp_->jerks);
        // getZone(zone,imp_->zones); 
        // std::cout << "/----------------vel acc zone------------------/" << std::endl;
        // aris::dynamic::dsp(1,imp_->vels.size(),imp_->vels.data());
        // aris::dynamic::dsp(1,imp_->accs.size(),imp_->accs.data());
        // aris::dynamic::dsp(1,imp_->jerks.size(),imp_->jerks.data());
        // aris::dynamic::dsp(1,imp_->zones.size(),imp_->zones.data());







    }
    auto FollowerCartParam::tool0s()->std::vector<aris::dynamic::Marker*>{ return imp_->tool0;}
    auto FollowerCartParam::tools()->std::vector<aris::dynamic::Marker*>{ return imp_->tool;}
    auto FollowerCartParam::wobj0s()->std::vector<aris::dynamic::Marker*>{ return imp_->wobj0;}
    auto FollowerCartParam::wobjs()->std::vector<aris::dynamic::Marker*>{ return imp_->wobj;}
    auto FollowerCartParam::getSpeed(const std::vector<std::vector<double>>& v,std::vector<double>& vel,std::vector<double>& acc,std::vector<double>& jerk)->void
    {
        auto& tgm = kaanh::multi_motion::TrajectoryGeneratorManager::instance();
        auto& mvp = kaanh::multi_motion::MoveParam::instance();
        auto& twm = kaanh::multi_motion::ToolWobjManager::instance();
        auto& cs = aris::server::ControlServer::instance();
        auto mm = dynamic_cast<aris::dynamic::MultiModel*>(&aris::server::ControlServer::instance().model());
        for (size_t i = 0; i < ctrlEes().size(); i++)
        {
            auto model = twm.getModel(ctrlEeId()[i]);
            auto line_vel = v.at(ctrlEeId()[i])[0];
            auto angle_vel = v.at(ctrlEeId()[i])[1];

            auto max_line_vel = mvp.getSpeedLimit().v_tcp_max;
            auto max_line_acc = max_line_vel * mvp.getAccDecJerkLimit().cart_line_acc;
            auto max_line_jerk = max_line_acc * mvp.getAccDecJerkLimit().cart_line_jerk;
            auto max_angle_vel = mvp.getSpeedLimit().w_tcp_max;
            auto max_angle_acc = max_angle_vel * mvp.getAccDecJerkLimit().cart_angle_acc;
            auto max_angle_jerk = max_angle_acc * mvp.getAccDecJerkLimit().cart_angle_jerk;
            auto line_vel_per = line_vel / max_line_vel;
            auto angle_vel_per = angle_vel / max_angle_vel;

            if (line_vel > mvp.getSpeedLimit().v_tcp_max || line_vel < mvp.getSpeedLimit().v_tcp_min)
            {
                throw std::runtime_error(TR_STR("Input speed.v_tcp is out of range"));
            }
            if (angle_vel > mvp.getSpeedLimit().w_tcp_max || angle_vel < mvp.getSpeedLimit().w_tcp_min)
            {
                throw std::runtime_error(TR_STR("Input speed.w_tcp is out of range"));
            }

            switch (imp_->ctrl_ee_types[i])
            {
            case aris::dynamic::EEType::PE313:[[fallthrough]];
            case aris::dynamic::EEType::PE321:[[fallthrough]];
            case aris::dynamic::EEType::PE123:[[fallthrough]];
            case aris::dynamic::EEType::PM:[[fallthrough]];
            case aris::dynamic::EEType::PQ:[[fallthrough]];
            case aris::dynamic::EEType::XYZT:[[fallthrough]];
            case aris::dynamic::EEType::XYT:
            {
                vel.push_back(line_vel); //移动速度
                vel.push_back(angle_vel); //转动速度
                acc.push_back(max_line_acc * std::sqrt(line_vel_per));//移动加速度
                acc.push_back(max_angle_acc * std::sqrt(angle_vel_per));//转动加速度
                jerk.push_back(max_line_jerk * std::pow(line_vel_per,1/3));
                jerk.push_back(max_angle_jerk * std::pow(angle_vel_per,1/3));
                break;
            }
            case aris::dynamic::EEType::RE123:[[fallthrough]];
            case aris::dynamic::EEType::RE313:[[fallthrough]];
            case aris::dynamic::EEType::RE321:[[fallthrough]];
            case aris::dynamic::EEType::RQ:[[fallthrough]];
            case aris::dynamic::EEType::RM:
            {
                vel.push_back(angle_vel); //转动速度
                acc.push_back(max_angle_acc * std::sqrt(angle_vel_per));//转动加速度
                jerk.push_back(max_angle_jerk * std::pow(angle_vel_per,1/3));
                break;
            }
            case aris::dynamic::EEType::A:
            {
                if (model->name() == "ExAxisModel")
                {
                    auto motion_id = mm->getMotionIds(std::vector<aris::Size>{twm.getModelId(ctrlEeId()[i])});
                    if (angle_vel > cs.controller().motorPool()[motion_id[0]].maxVel())
                    {
                        throw std::runtime_error(TR_STR("ext vel is out of range"));
                    }
                    
                    vel.push_back(angle_vel); //转动速度
                    acc.push_back(cs.controller().motorPool()[motion_id[0]].maxAcc()*0.9999);//加速度设置为电机性能
                    jerk.push_back(mvp.getAccDecJerkLimit().cart_angle_jerk * cs.controller().motorPool()[motion_id[0]].maxAcc()); //最大加速度*倍率
                }
                else
                {
                    vel.push_back(angle_vel); //转动速度
                    acc.push_back(max_angle_acc * std::sqrt(angle_vel_per));//转动加速度
                    jerk.push_back(max_angle_jerk * std::pow(angle_vel_per,1/3));
                }                

                break;
            }
            case aris::dynamic::EEType::XYZ:[[fallthrough]];
            case aris::dynamic::EEType::XY:
            {
                vel.push_back(line_vel); //移动速度
                acc.push_back(max_line_acc * std::sqrt(line_vel_per));//移动加速度
                jerk.push_back(max_line_jerk * std::pow(line_vel_per,1/3));
                break;
            }
            case aris::dynamic::EEType::X:
            {
                if (model->name() == "ExAxisModel")
                {
                    auto motion_id = mm->getMotionIds(std::vector<aris::Size>{twm.getModelId(ctrlEeId()[i])});
                    if (line_vel > cs.controller().motorPool()[motion_id[0]].maxVel())
                    {
                        throw std::runtime_error(TR_STR("ext vel is out of range"));
                    }
                    
                    vel.push_back(line_vel); //转动速度
                    acc.push_back(cs.controller().motorPool()[motion_id[0]].maxAcc());//加速度设置为电机性能
                    jerk.push_back(mvp.getAccDecJerkLimit().cart_line_jerk * cs.controller().motorPool()[motion_id[0]].maxAcc()); //最大加速度*倍率
                }
                else
                {
                    vel.push_back(line_vel); //移动速度
                    acc.push_back(max_line_acc * std::sqrt(line_vel_per));//移动加速度
                    jerk.push_back(max_line_jerk * std::pow(line_vel_per,1/3));
                }

                break;
            }
            default:
                break;
            }
        }
    }
    auto FollowerCartParam::getZone(const std::vector<std::vector<double>>& zone,std::vector<double>& zone_out)->void
    {
        for (size_t i = 0; i < ctrlEes().size(); i++)
        {
            switch (imp_->ctrl_ee_types[i])
            {
            case aris::dynamic::EEType::PE313:[[fallthrough]];
            case aris::dynamic::EEType::PE321:[[fallthrough]];
            case aris::dynamic::EEType::PE123:[[fallthrough]];
            case aris::dynamic::EEType::PM:[[fallthrough]];
            case aris::dynamic::EEType::PQ:[[fallthrough]];
            case aris::dynamic::EEType::XYZT:[[fallthrough]];
            case aris::dynamic::EEType::XYT:
            {
                zone_out.push_back(zone.at(ctrlEeId()[i])[0]); //平动转弯区
                zone_out.push_back(zone.at(ctrlEeId()[i])[1]); //转动转弯区
                break;
            }
            case aris::dynamic::EEType::RE123:[[fallthrough]];
            case aris::dynamic::EEType::RE313:[[fallthrough]];
            case aris::dynamic::EEType::RE321:[[fallthrough]];
            case aris::dynamic::EEType::RQ:[[fallthrough]];
            case aris::dynamic::EEType::RM:[[fallthrough]];
            case aris::dynamic::EEType::A:
            {
                zone_out.push_back(zone.at(ctrlEeId()[i])[1]); //转动转弯区
                break;
            }
            case aris::dynamic::EEType::XYZ:[[fallthrough]];
            case aris::dynamic::EEType::XY:[[fallthrough]];
            case aris::dynamic::EEType::X:
            {
                zone_out.push_back(zone.at(ctrlEeId()[i])[0]); //外部轴弯区
                break;
            }
            default:
                break;
            }
        }
    }
    auto FollowerCartParam::getPe(const std::vector<std::tuple<std::vector<double>, std::vector<int>, std::vector<int>>>& pe,std::vector<std::vector<double>>& pe_out)->void
    {
        for (size_t i = 0; i < ctrlEeId().size(); i++)
        {
            auto& pe_data = pe.at(ctrlEeId()[i]);
            auto&    ee_pe = std::get<0>(pe_data);
            auto&       tw = std::get<1>(pe_data);
            auto& confdata = std::get<2>(pe_data);

            // imp_->tool_id.push_back(tw[0]);
            // imp_->wobj_id.push_back(tw[1]);
            // imp_->is_ext_tool.push_back(tw[2]);
            // imp_->pe_confdata.push_back(confdata);
            pe_out.push_back(ee_pe);
        }
    }
    auto FollowerCartParam::setPe(const double*cur_pe,aris::plan::Plan* plan)->int
    {
        aris::Size ee_pos_id = 0;
        for (size_t i = 0; i < ctrlEes().size(); i++)
        {
            auto &model = const_cast<aris::dynamic::MultiModel*>(multiModel())->subModels()[twManager()->getModelId(ctrlEeId()[i])];
            switch (ctrlEeTypes()[i])
            {
            case aris::dynamic::EEType::PE313:[[fallthrough]];
            case aris::dynamic::EEType::PE321:[[fallthrough]];
            case aris::dynamic::EEType::PE123:[[fallthrough]];
            case aris::dynamic::EEType::PM:[[fallthrough]];
            case aris::dynamic::EEType::PQ:
            {
                if (!imp_->is_ext_tool[i]) 
                {   
                    imp_->tool[i]->setPe(*imp_->wobj[i], cur_pe + ee_pos_id, "321");
                }
                else // 外部工具 
                {
                    double pe[6],pm[16],pm_target[16];
                    aris::dynamic::s_vc(6,cur_pe+ee_pos_id,pe);
                    aris::dynamic::s_pe2pm(pe,pm,"321");
                    aris::dynamic::s_inv_pm(pm,pm_target);
                    imp_->hd_wobj[i]->setPm(*imp_->ex_tool[i],pm_target);
                }
                
                ctrlEes().at(i)->updP();
                ee_pos_id += 6;
                break;
            }
            case aris::dynamic::EEType::RE123:[[fallthrough]];
            case aris::dynamic::EEType::RE313:[[fallthrough]];
            case aris::dynamic::EEType::RE321:[[fallthrough]];
            case aris::dynamic::EEType::RQ:[[fallthrough]];
            case aris::dynamic::EEType::RM:
            {

                double ee_pos[6] = {0.0,0.0,0.0,0.0,0.0,0.0};
                aris::dynamic::s_vc(3,cur_pe+ee_pos_id,ee_pos+3);

                if (!imp_->is_ext_tool[i]) 
                {   
                    imp_->tool[i]->setPe(*imp_->wobj[i], ee_pos, "123");
                }else{}
            
                ctrlEes().at(i)->updP();
                ee_pos_id += 6;
                break;
            }
            case aris::dynamic::EEType::XYZ:
            {
                double ee_pos[6] = {0.0,0.0,0.0,0.0,0.0,0.0};
                s_vc(3,cur_pe+ee_pos_id,ee_pos);  
                
                if (!imp_->is_ext_tool[i]) 
                { 
                    imp_->tool[i]->setPe(*imp_->wobj[i], ee_pos, "321");
                }   else{}       
                
                ctrlEes().at(i)->updP();
                ee_pos_id += 3;
                break;
            }
            case aris::dynamic::EEType::XYZT:
            {
                std::vector<double> ee_pos(6,0);
                ee_pos[0] = cur_pe[ee_pos_id];
                ee_pos[1] = cur_pe[ee_pos_id+1];
                ee_pos[2] = cur_pe[ee_pos_id+2];
                ee_pos[3] = cur_pe[ee_pos_id+3];
                ee_pos[4] = 0;
                ee_pos[5] = 0;
                
                if (!imp_->is_ext_tool[i]) 
                {   
                    imp_->tool[i]->setPe(*imp_->wobj[i], ee_pos.data(), "321");
                }else{}

                ctrlEes().at(i)->updP();
                ee_pos_id += 4;
                break;
            }
            case aris::dynamic::EEType::A:[[fallthrough]];
            case aris::dynamic::EEType::X:
            {
                ctrlEes().at(i)->setP(cur_pe + ee_pos_id);
                ee_pos_id++;
                break;
            }
            default:
                break;
            }
            
            //-------------------------求运动学反解----------------------------//
            if (model.isSingular(0.05))
            {
                log()->setPlanExeError(int(ErrorCode2::eCloseToSingularPoints),*plan,TR_STR("Robot '{1}'",std::to_string(twManager()->getModelId(ctrlEeId()[i]))));
                return -int(ErrorCode2::eCloseToSingularPoints);
            }


            if (model.inverseKinematics())
            {
                log()->setPlanExeError(int(ErrorCode2::eInverseKinematics),*plan);
                return -int(ErrorCode2::eInverseKinematics);
            }
        }
        return 0;
    }
    auto FollowerCartParam::getPe(const std::vector<std::vector<double>>& pe_in,std::vector<double>& pe_out)->void
    {
        for (auto& pos: pe_in)
        {
            for (size_t i = 0; i < pos.size(); i++)
            {
                pe_out.push_back(pos[i]);
            }
        }
    }
    auto FollowerCartParam::getInitPe()->std::vector<std::vector<double>>
    {
        std::vector<std::vector<double>> init_pe;
        // init_pe.resize(ctrlEes().size());
        // imp_->init_confdata.resize(ctrlEes().size());
        // for (size_t i = 0; i < ctrlEes().size(); i++)
        // {
        //     init_pe.at(i).resize(ctrlEes()[i]->dim());
        // }
        
        // for (size_t i = 0; i < ctrlEeId().size(); i++)
        // {
        //     if (!imp_->is_ext_tool[i])
        //     {
        //         kaanh::multi_motion::EeManager::instance().getPe(imp_->tool,imp_->wobj,ctrlEeId(),init_pe.data(),imp_->init_confdata.data(),true);
        //     }
        //     else
        //     {
        //         kaanh::multi_motion::EeManager::instance().getPe(imp_->ex_tool,imp_->hd_wobj,ctrlEeId(),init_pe.data(),imp_->init_confdata.data(),true);
        //     }
        // }
        
        return init_pe;
    }
    auto FollowerCartParam::getToolWobj()->void
    {
        // 获取tool0 wobj0
        std::vector<aris::Size> tool0_ids(ctrlEeId().size(),0);
        std::vector<aris::Size> wobj0_ids(ctrlEeId().size(),0);
        imp_->tool0 = twManager()->getTools(ctrlEeId(),tool0_ids);
        imp_->wobj0 = twManager()->getWobjs(ctrlEeId(),wobj0_ids);
        
        for (size_t i = 0; i < ctrlEeId().size(); i++)
        {
            // 工具工件

            imp_->tool_id.push_back(0);
            imp_->wobj_id.push_back(0);
            imp_->is_ext_tool.push_back(false);
            // imp_->pe_confdata.push_back(confdata);

            if (moveParam()->getCouplingConfig().is_coupling) // 耦合控制 仅支持2个模型的情形
            {
                switch (moveParam()->getCouplingConfig().type)
                {
                case CouplingType::kToolTool :
                {
                    if (!imp_->is_ext_tool[i])
                    {
                        if (i == 1) // 6轴等 i== 1  ee_id=0
                        {
                            imp_->tool.push_back(twManager()->getTool(ctrlEeId()[i],imp_->tool_id[i]));
                            imp_->wobj.push_back(twManager()->getTool(ctrlEeId()[i]+1,imp_->wobj_id[i]));
                        }
                        else // i == 0 外部轴 eeid=1
                        {
                            imp_->tool.push_back(twManager()->getTool(ctrlEeId()[i],imp_->tool_id[i]));
                            imp_->wobj.push_back(twManager()->getWobj(ctrlEeId()[i],imp_->wobj_id[i]));
                        }
                    }
                    else // 外部工具情形需求不明确，暂不支持
                    {
                        throw std::runtime_error(TR_STR("Not supported this mode"));
                        // imp_->ex_tool.push_back(twManager()->getExtTool(ctrlEeId()[i],tool_id[i]-1));
                        // imp_->hd_wobj.push_back(twManager()->getHdWobj(ctrlEeId()[i],wobj_id[i])-1);
                    }
                    break;
                }
                default:
                    break;
                }
            }
            else
            {
                // 获取工具工件            
                if (!imp_->is_ext_tool[i])
                {
                    imp_->tool.push_back(twManager()->getTool(ctrlEeId()[i],imp_->tool_id[i]));
                    imp_->wobj.push_back(twManager()->getWobj(ctrlEeId()[i],imp_->wobj_id[i]));
                }
                else
                {
                    imp_->ex_tool.push_back(twManager()->getExtTool(ctrlEeId()[i],imp_->tool_id[i]-1));
                    imp_->hd_wobj.push_back(twManager()->getHdWobj(ctrlEeId()[i],imp_->wobj_id[i])-1);
                }
            }
        }
    }
    FollowerCartParam::FollowerCartParam() : imp_(new Imp) {}
    FollowerCartParam::~FollowerCartParam() = default;

    struct UDPFollower::Imp{
        aris::core::CommandParser parser_;

        std::recursive_mutex mutex_;
        std::atomic_bool is_running {false};

        std::unique_ptr<aris::core::PointerArray<FollowerParamBase>> follower_param{new aris::core::PointerArray<FollowerParamBase>};
        FollowerMode mode_{FollowerMode::kNone};
    };
    UDPFollower::UDPFollower() : imp_(new Imp) {}
    UDPFollower::~UDPFollower() = default;
    auto UDPFollower::init()->void {

        for (auto& follower_param : followerParamPool())
        {
            follower_param.init();
        }
        

        {
            Command cmd;
            aris::core::fromXmlString(cmd,
                "<Command name=\"start_follower\">"
                "</Command>");
            imp_->parser_.commandPool().push_back(cmd);
        }

        {
            Command cmd;
            aris::core::fromXmlString(cmd,
                "<Command name=\"stop_follower\">"
                "</Command>");
            imp_->parser_.commandPool().push_back(cmd);
        }

        imp_->parser_.init();

        auto get_cbk = [&](aris::server::ControlServer& cs)->nlohmann::json
        {
            nlohmann::json ret;
            ret["running_state"] = isRunning();
            ret["follower_mode"] = getFollowerMode();
            return ret;
        };

        // 注册robot回调函数
	    auto &robot = kaanh::robot::Robot::instanceInCs();
	    // 注册get 返回需要周期性实时返回的数据
        robot.registerGetFunc(get_cbk, "follower_msg", kaanh::robot::Robot::GetType::kInfo);
    
    };
    auto UDPFollower::followerParamPool()->aris::core::PointerArray<FollowerParamBase>& { return *imp_->follower_param; }
    auto UDPFollower::resetFollowerParamPool(aris::core::PointerArray<FollowerParamBase> *pool)->void { imp_->follower_param.reset(pool); }
    auto UDPFollower::setRunningState(const bool val)->void { imp_->is_running.store(val);}
    auto UDPFollower::isRunning()->bool { return imp_->is_running.load(); }
    auto UDPFollower::setFollowerMode(const FollowerMode mode)->void 
    {
        std::lock_guard<std::recursive_mutex> lck(imp_->mutex_); 
        imp_->mode_ = mode;
    }
    auto UDPFollower::getFollowerMode() -> const FollowerMode& 
    { 
        std::lock_guard<std::recursive_mutex> lck(imp_->mutex_); 
        return imp_->mode_;
    }
    auto UDPFollower::execute(const std::string &str, std::function<void(std::string)> send_ret) noexcept->std::pair<std::string, std::string> {
        std::string_view cmd_name;
        std::map<std::string_view, std::string_view> params;

        try {
            std::tie(cmd_name, params) = imp_->parser_.parse(str);
        } catch (const std::exception &err) {
            sendRet(send_ret, -1, TR_STR("invalid command '{1}'",str));
            END_CMD_FLOW;
        }

        try {

            if (cmd_name == "start_follower") {
                setRunningState(true);
                INFO_OPR(TR_STR("start follower"), 0);
                sendRet(send_ret,0);
            }
            else if (cmd_name == "stop_follower") {
                setRunningState(false);
                INFO_OPR(TR_STR("stop follower"), 0);
                sendRet(send_ret,0);
            }
            else {
                throw std::runtime_error(TR_STR("invalid command '{1}'", str));
            }
        } catch (const std::exception &err) {
            sendRet(send_ret, -1, err.what());
        }

        END_CMD_FLOW;
    }
    auto UDPFollower::instanceInCs()->UDPFollower& {
        return *dynamic_cast<UDPFollower*>(&(kaanh::middleware::Shell::instanceInCs().getModule("UDPFollower")));
    }


    struct FollowerAjTg::Imp
    {
        kaanh::follower::UDPFollower* udpf = nullptr;
        aris::dynamic::MultiModel* multi_model = nullptr;
        FollowerAjParam* aj_param = nullptr;
        kaanh::robot::Robot *robot{nullptr};
        std::vector<double> cur_pos,init_pos;
        int param_id = 0;
    };
    auto FollowerAjTg::prepareNrt() -> void
    {
        imp_->udpf = &kaanh::follower::UDPFollower::instanceInCs(); 
        imp_->udpf->setRunningState(true);
        imp_->udpf->setFollowerMode(FollowerMode::kAixs);
        imp_->multi_model = dynamic_cast<aris::dynamic::MultiModel*>(&aris::server::ControlServer::instance().model());
        imp_->robot = &kaanh::robot::Robot::instanceInCs();
        imp_->aj_param = dynamic_cast<FollowerAjParam*>(&imp_->udpf->followerParamPool()[imp_->param_id]);
        imp_->cur_pos.resize(imp_->aj_param->ctrlMotions().size(),0); // 初始化实时循环pos大小
        
        imp_->aj_param->setLastId(0);
        imp_->aj_param->tg()->clearAllPos();
        imp_->init_pos.resize(imp_->aj_param->ctrlMotionId().size(), 0);
        for (size_t i = 0; i < imp_->aj_param->ctrlMotionId().size(); i++)
        {
            imp_->aj_param->ctrlMotions()[i]->getP(imp_->init_pos.data() + i);
        }
        imp_->aj_param->tg()->insertInitPos(0, imp_->init_pos.data());
            
        // 7.使用USE_TARGET_POS
        for (size_t i = 0, j = 0; i < imp_->aj_param->ctrlMotionId().size(); i++)
        {
            auto &option = motorOptions()[imp_->aj_param->ctrlMotionId()[i]];
            option |= USE_TARGET_POS | NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
        }

 
    }
    auto FollowerAjTg::executeRT() -> int
    {

        //-------------------------是否暂停-----------------------//
        if (!imp_->udpf->isRunning())
        {
            if (imp_->aj_param->tg()->currentDs() < 1e-10 || imp_->aj_param->tg()->leftTotalS() < 1e-10)
                return 0;
        
           imp_->aj_param->tg()->setTargetDs(0);
        }
        else
        {
            if (imp_->robot->robotOpMode() == kaanh::robot::RobotOpMode::kManual)       
            {
                imp_->aj_param->tg()->setTargetDs(imp_->robot->vel() / 100.0);
            }    
            else
                imp_->aj_param->tg()->setTargetDs(imp_->robot->pgmVel() / 100.0);                
        }
        if (count()%100 == 0)
        {
            mout() << "waiting..." << imp_->aj_param->tg()->leftTotalS()<< std::endl;
        }
        

        if (imp_->aj_param->tg()->leftTotalS() < 1e-10)
            return 1;
        
        //-------------------------获取规划值-----------------------//
//        aris::dynamic::dsp(1,imp_->cur_pos.size(),imp_->cur_pos.data());
        auto ret =imp_->aj_param->tg()->getEePosAndMoveDt(imp_->cur_pos.data());
       

        //-------------------------设置末端-----------------------//
        for (size_t i = 0; i < imp_->aj_param->ctrlMotions().size(); i++)
        {
            imp_->aj_param->ctrlMotions()[i]->setP(imp_->cur_pos.data() + i);
        }

        imp_->multi_model->forwardKinematics();
        imp_->multi_model->updP();
        imp_->multi_model->forwardKinematicsVel();
        

        return 1;
    }
    auto FollowerAjTg::collectNrt()->void
    {
        //-------------------------在没有取消的情况下 暂停 急停 正常结束 保存当前电机位置----------------------------//

        if (this->executeRetCode() < 0)
        {
            imp_->aj_param->tg()->setTargetDs(0);
            imp_->aj_param->tg()->setCurrentDs(0);
            imp_->aj_param->tg()->setCurrentDds(0);
        }
    
        if(imp_->aj_param->tg()) imp_->aj_param->tg()->clearUsedPos();
        // imp_->udpf->setFollowerMode(FollowerMode::kNone);
        imp_->udpf->setRunningState(false);
    } 
    FollowerAjTg::~FollowerAjTg() = default;
    FollowerAjTg::FollowerAjTg(const std::string &name) : imp_(new Imp)
    {
        aris::core::fromXmlString(command(),
                                  "<Command name=\"follower_aj\">"
                                  "	<GroupParam>"
                                  "		<Param name=\"tg\" />"
                                  "		<Param name=\"continuous_id\" default=\"-1\"/>"
                                  "		<Param name=\"reinit\" default=\"0\" />"
                                  "	</GroupParam>"
                                  "</Command>");
    }
    KAANH_DEFINE_BIG_FOUR_CPP(FollowerAjTg)

    struct FollowerAj::Imp
    {
        kaanh::follower::UDPFollower* udpf = nullptr;
        aris::dynamic::MultiModel* multi_model = nullptr;
        FollowerAjParam* aj_param = nullptr;
        kaanh::robot::Robot *robot{nullptr};
        std::vector<double> cur_pos,init_pos;
        int param_id = 0;
    };
    auto FollowerAj::prepareNrt() -> void
    {
        imp_->udpf = &kaanh::follower::UDPFollower::instanceInCs();
        imp_->udpf->setRunningState(true);
        imp_->udpf->setFollowerMode(FollowerMode::kAixs);
        imp_->multi_model = dynamic_cast<aris::dynamic::MultiModel*>(&aris::server::ControlServer::instance().model());
        imp_->robot = &kaanh::robot::Robot::instanceInCs();
        imp_->aj_param = dynamic_cast<FollowerAjParam*>(&imp_->udpf->followerParamPool()[imp_->param_id]);
        imp_->cur_pos.resize(imp_->aj_param->ctrlMotions().size(),0); // 初始化实时循环pos大小
        std::vector<double> pos;
        // if(!imp_->udpf->ajDataToQueue().isEmpty()) imp_->udpf->clearAjDataQueue();
        while (!imp_->aj_param->ajDataToQueue().isEmpty())
        {
         
            imp_->aj_param->getAjDataFromQueue(pos);
            mout()<< "clear old data ...." << std::endl;   
        }
       
        imp_->aj_param->setLastId(0);
        imp_->init_pos.resize(imp_->aj_param->ctrlMotionId().size(), 0);
        for (size_t i = 0; i < imp_->aj_param->ctrlMotionId().size(); i++)
        {
            imp_->aj_param->ctrlMotions()[i]->getP(imp_->init_pos.data() + i);
        }
            
        // 7.使用USE_TARGET_POS
        for (size_t i = 0, j = 0; i < imp_->aj_param->ctrlMotionId().size(); i++)
        {
            auto &option = motorOptions()[imp_->aj_param->ctrlMotionId()[i]];
            option |= USE_TARGET_POS | NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
        }
    }
    auto FollowerAj::executeRT() -> int
    {

        //-------------------------是否暂停-----------------------//
        if (!imp_->udpf->isRunning())
        {
            return 0;
        }

        
        if (!imp_->aj_param->getAjDataFromQueue(imp_->cur_pos))
        {           
            if(count()%100 == 0)
                mout()<< "waiting...." << std::endl;
            return 1;
        }

        aris::dynamic::dsp(1,imp_->cur_pos.size(),imp_->cur_pos.data());

        //-------------------------设置末端-----------------------//
        for (size_t i = 0; i < imp_->aj_param->ctrlMotions().size(); i++)
        {
            imp_->aj_param->ctrlMotions()[i]->setP(imp_->cur_pos.data() + i);
        }

        imp_->multi_model->forwardKinematics();
        imp_->multi_model->updP();
        imp_->multi_model->forwardKinematicsVel();
        

        return 1;
    }
    auto FollowerAj::collectNrt()->void
    {
        // imp_->udpf->setFollowerMode(FollowerMode::kNone);
        imp_->udpf->setRunningState(false);
    } 
    FollowerAj::~FollowerAj() = default;
    FollowerAj::FollowerAj(const std::string &name) : imp_(new Imp)
    {
        aris::core::fromXmlString(command(),
                                  "<Command name=\"follower_aj\">"
                                  "	<GroupParam>"
                                  "		<Param name=\"tg\" />"
                                  "		<Param name=\"continuous_id\" default=\"-1\"/>"
                                  "		<Param name=\"reinit\" default=\"0\" />"
                                  "	</GroupParam>"
                                  "</Command>");
    }
    KAANH_DEFINE_BIG_FOUR_CPP(FollowerAj)

    struct FollowerAjAbsolute2::Imp{
        kaanh::follower::UDPFollower* udp_follower = nullptr;
        kaanh::robot::Robot *robot{nullptr};
        FollowerAjParam *aj_param = nullptr;
        int param_id = 0;
        aris::dynamic::MultiModel* multi_model = nullptr;
        std::vector<double> target_pos;
        std::vector<double> cur_pos_;

        int vel_percent = 10;
        std::vector<double> vels;
        std::vector<double> accs;
        std::vector<double> jerks;
        std::vector<double> zones;
        std::vector<double> now_vel,now_acc;
        int ret = 0;
        int dt = 1;
    };
    auto FollowerAjAbsolute2::prepareNrt()->void{
        imp_->udp_follower = &kaanh::follower::UDPFollower::instanceInCs();
        imp_->robot = &kaanh::robot::Robot::instanceInCs();
        imp_->udp_follower->setRunningState(true);
        imp_->udp_follower->setFollowerMode(FollowerMode::kAixs);
        imp_->multi_model = dynamic_cast<aris::dynamic::MultiModel*>(&aris::server::ControlServer::instance().model());
        imp_->aj_param = dynamic_cast<FollowerAjParam*>(&imp_->udp_follower->followerParamPool()[imp_->param_id]);
        auto& mvp = kaanh::multi_motion::MoveParam::instance();
        imp_->cur_pos_.resize(imp_->aj_param->ctrlMotionId().size(), 0);
        imp_->target_pos.resize(imp_->aj_param->ctrlMotionId().size(), 0);

        imp_->dt = master()->samplePeriodNs() / 1000000;
        for(int i = 0; i< imp_->cur_pos_.size(); i++){
            imp_->aj_param->ctrlMotions()[i]->getP(imp_->cur_pos_.data() + i);
            imp_->aj_param->ctrlMotions()[i]->getP(imp_->target_pos.data() + i);
        }

        for (size_t i = 0; i < imp_->aj_param->ctrlMotionId().size(); i++)
        {
            imp_->vel_percent = int32Param("vel_percent");
            imp_->vel_percent /= 100.0;
            imp_->vels.push_back(0.999999 * controller()->motorPool()[imp_->aj_param->ctrlMotionId()[i]].maxVel() * imp_->vel_percent);
            imp_->accs.push_back(0.999999 * controller()->motorPool()[imp_->aj_param->ctrlMotionId()[i]].maxAcc() * std::sqrt(imp_->vel_percent));
            imp_->jerks.push_back(0.999999 * controller()->motorPool()[imp_->aj_param->ctrlMotionId()[i]].maxAcc() * mvp.getAccDecJerkLimit().joint_jerk * std::pow(imp_->vel_percent,1/3));
            imp_->zones.push_back(10);

            imp_->now_vel.push_back(0.0);
            imp_->now_acc.push_back(0.0);
        }

        imp_->aj_param->clearAjDataQueue();
        // TODO 获取当前位置姿态
        for (size_t i = 0, j = 0; i < imp_->aj_param->ctrlMotionId().size(); i++)
        {
            auto &option = motorOptions()[imp_->aj_param->ctrlMotionId()[i]];
            option |= USE_TARGET_POS | NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
        }
    }
    auto FollowerAjAbsolute2::executeRT()->int{

	// 规划算法
	imp_->ret = 0;

	if(!imp_->udp_follower->isRunning())
	{
        for(std::size_t i = 0;i < imp_->aj_param->ctrlMotionId().size();i++)
        {
            double p_next{0.0 }, v_next{0.0}, a_next{0.0};
            aris::Size t;


            imp_->ret += aris::plan::moveAbsolute2(imp_->cur_pos_[i], imp_->now_vel[i], imp_->now_acc[i]
                , imp_->target_pos[i] , 0.0, 0.0
                , imp_->vels[i], imp_->accs[i], imp_->accs[i]
                , imp_->dt * 1e-3, 1e-10, p_next, v_next, a_next, t);
        

                    imp_->cur_pos_[i] = p_next;
					imp_->now_vel[i] = v_next;
					imp_->now_acc[i] = a_next;
        }

        if (!imp_->ret) return imp_->ret;
        
	}
	else //正常运行
	{
        imp_->aj_param->getAjDataFromQueue(imp_->target_pos);
        for(std::size_t i = 0;i < imp_->aj_param->ctrlMotionId().size();i++)
        {
            double p_next{0.0 }, v_next{0.0}, a_next{0.0};
            aris::Size t;

            double real_vel = 0;
            if (imp_->aj_param->robot()->robotOpMode() == kaanh::robot::RobotOpMode::kManual)
                real_vel = imp_->aj_param->robot()->vel() / 100.0;
            else
                real_vel = imp_->aj_param->robot()->pgmVel() / 100.0;

            imp_->ret += aris::plan::moveAbsolute2(imp_->cur_pos_[i], imp_->now_vel[i], imp_->now_acc[i]
                , imp_->target_pos[i] , 0.0, 0.0
                , real_vel * imp_->vels[i], imp_->accs[i], imp_->accs[i]
                , imp_->dt * 1e-3, 1e-10, p_next, v_next, a_next, t);
    

            imp_->cur_pos_[i] = p_next;
            imp_->now_vel[i] = v_next;
            imp_->now_acc[i] = a_next;
        }
    
        if (!imp_->ret)
        {           
            if(count()%100 == 0)
                mout()<< "waiting...." << std::endl;
            return 1;
        }
	}

        // 设置电机位置
        for (size_t i = 0; i < imp_->aj_param->ctrlMotions().size(); i++)
        {
            imp_->aj_param->ctrlMotions()[i]->setP(imp_->cur_pos_.data() + i);
        }
        // 更新模型
        imp_->multi_model->forwardKinematics();
        imp_->multi_model->updP();
        imp_->multi_model->forwardKinematicsVel();
        return 1;
    }
    auto FollowerAjAbsolute2::collectNrt()->void{
        // imp_->udp_follower->setFollowerMode(FollowerMode::kNone);
        imp_->udp_follower->setRunningState(false);
    }
    FollowerAjAbsolute2::~FollowerAjAbsolute2() = default;
    FollowerAjAbsolute2::FollowerAjAbsolute2(const std::string &name): imp_(new Imp){
        aris::core::fromXmlString(command(),
                                  "<Command name=\"follower_aj_abs2\">"
                                  "	<GroupParam>"
                                  "        <Param name=\"max_vel\" default=\"1.5\"/>"
                                  "        <Param name=\"max_acc\" default=\"1.5\"/>"
                                  "        <Param name=\"max_dec\" default=\"1.5\"/>"
                                  "        <Param name=\"max_jerk\" default=\"5\"/>"
                                  "        <Param name=\"vel_precent\" default=\"10\"/>"
                                  "	</GroupParam>"
                                  "</Command>");
    }
    KAANH_DEFINE_BIG_FOUR_CPP(FollowerAjAbsolute2)
    

    // 示教运动--笛卡尔空间点动 //
struct FollowerCart::Imp 
{
	// 全局变量
	kaanh::multi_motion::JogControl* jog_param{nullptr};
    kaanh::follower::UDPFollower* udpf= nullptr;
    FollowerCartParam *cart_param = nullptr;
    int param_id = 1;

	// 控制参数
    std::vector<std::vector<double>> vel,acc,dec,jerk,step;
    std::vector<std::vector<double>> now_vel,now_acc;
    std::vector<std::vector<double>> cur_pos,now_pos,target_pos;

    // 运动模式和坐标系
	kaanh::robot::JogCoordinate cor;
    int dt;

	// 工具工具 外部工具手持工件
	std::vector<aris::dynamic::Marker*> tool, tool0,ex_tool,hd_wobj, wobj, wobj0;
	std::vector<Size> tool_id, wobj_id;


	// 规划用的变量
	int ret = 0;

	double pm_now[16], pm_temp[16],pm[16],pm_target[16]{0};
};
auto FollowerCart::getControlParam()->void
{
    // 解析参数 vel acc dec
    for (size_t i = 0; i < imp_->cart_param->ctrlEes().size(); i++)
    {
        std::vector<double> vel,acc,dec,jerk;
        for(aris::Size j = 0; j < 6; j++)
        {
            if(j<3)
            {
                vel.push_back(imp_->jog_param->JogCParamPool()[imp_->cart_param->ctrlEeId()[i]].jogCVel()[j]/1000.0);
                acc.push_back(imp_->jog_param->JogCParamPool()[imp_->cart_param->ctrlEeId()[i]].jogCAcc()[j]/1000.0);
                jerk.push_back(imp_->jog_param->JogCParamPool()[imp_->cart_param->ctrlEeId()[i]].jogCJerk()[j]*acc[j]);
                dec.push_back(imp_->jog_param->JogCParamPool()[imp_->cart_param->ctrlEeId()[i]].jogCDec()[j]/1000.0);
            }
            else
            {
                vel.push_back(imp_->jog_param->JogCParamPool()[imp_->cart_param->ctrlEeId()[i]].jogCVel()[j]*aris::PI/180.0);
                acc.push_back(imp_->jog_param->JogCParamPool()[imp_->cart_param->ctrlEeId()[i]].jogCAcc()[j]*aris::PI/180.0);
                jerk.push_back(imp_->jog_param->JogCParamPool()[imp_->cart_param->ctrlEeId()[i]].jogCJerk()[j]*acc[j]);
                dec.push_back(imp_->jog_param->JogCParamPool()[imp_->cart_param->ctrlEeId()[i]].jogCDec()[j]*aris::PI/180.0);
            }
        }
        imp_->vel.push_back(vel);
        imp_->acc.push_back(acc);
        imp_->dec.push_back(dec);
        imp_->jerk.push_back(jerk);
		imp_->tool_id.push_back(imp_->cart_param->robot()->choosenTool()[imp_->cart_param->ctrlEeId()[i]]);
		imp_->wobj_id.push_back(imp_->cart_param->robot()->choosenWobj()[imp_->cart_param->ctrlEeId()[i]]);
    }

	// tool wobj
    imp_->tool0 = imp_->cart_param->twManager()->getTools(imp_->cart_param->ctrlEeId(),std::vector<Size>(imp_->cart_param->ctrlEeId().size(),0));
    imp_->wobj0 = imp_->cart_param->twManager()->getWobjs(imp_->cart_param->ctrlEeId(),std::vector<Size>(imp_->cart_param->ctrlEeId().size(),0));
	for (size_t i = 0; i < imp_->cart_param->ctrlEeId().size(); i++)
	{
        if (imp_->cart_param->twManager()->toolMode()[imp_->cart_param->ctrlEeId()[i]] == kaanh::multi_motion::ToolMode::kTool)
		{
            imp_->tool.push_back(imp_->cart_param->twManager()->getTool(imp_->cart_param->ctrlEeId()[i],imp_->tool_id[i]));
            imp_->wobj.push_back(imp_->cart_param->twManager()->getWobj(imp_->cart_param->ctrlEeId()[i],imp_->wobj_id[i]));
		}
		else
		{
            throw std::runtime_error(TR_STR("Not currently supporting the use of external tools"));
            imp_->ex_tool.push_back(imp_->cart_param->twManager()->getExtTool(imp_->cart_param->ctrlEeId()[i],imp_->tool_id[i]));
            imp_->hd_wobj.push_back(imp_->cart_param->twManager()->getHdWobj(imp_->cart_param->ctrlEeId()[i],imp_->wobj_id[i]));
		}
	}

    imp_->now_vel.resize(imp_->cart_param->ctrlEeId().size(),std::vector<double>(6,0));
    imp_->now_acc.resize(imp_->cart_param->ctrlEeId().size(),std::vector<double>(6,0));
    imp_->target_pos.resize(imp_->cart_param->ctrlEeId().size(),std::vector<double>(6,0));
    imp_->now_pos.resize(imp_->cart_param->ctrlEeId().size(),std::vector<double>(6,0));
    imp_->cur_pos.resize(imp_->cart_param->ctrlEeId().size(),std::vector<double>(6,0));


}
auto FollowerCart::prepareNrt()->void 
{
	// 获取全局单例
	imp_->jog_param = &kaanh::multi_motion::JogControl::instanceInCs();


    imp_->udpf = &kaanh::follower::UDPFollower::instanceInCs();
    imp_->udpf->setRunningState(true);
    imp_->udpf->setFollowerMode(FollowerMode::kCart);
    imp_->cart_param = dynamic_cast<FollowerCartParam*>(&imp_->udpf->followerParamPool()[imp_->param_id]);

	
	// 坐标系
	imp_->cor = imp_->cart_param->robot()->jogCoordinate();
    if (imp_->cor == kaanh::robot::JogCoordinate::kAxis)
	{
		imp_->cart_param->log()->setPlanPreError(int(ErrorCode2::eJogCRunModeAtAxis),*this);
		option() |= aris::plan::Plan::Option::NOT_RUN_EXECUTE_FUNCTION;
		return;
	}
	
    imp_->dt = master()->samplePeriodNs() / 1000000;

	// 判断当前是否有指令在执行  单步和连续的逻辑
	std::shared_ptr<aris::plan::Plan> planptr = controlServer()->currentExecutePlan();

	try
	{
        imp_->cart_param->clearDataQueue();
        imp_->cart_param->setLastId(0);

        //当前有其他指令在执行//
        if (planptr) 
        {
            throw std::runtime_error("有指令正在执行");
            option() |= aris::plan::Plan::Option::NOT_RUN_EXECUTE_FUNCTION;
            return;
        }
        
        this->getControlParam();
		
	}
	catch(const std::exception& e)
	{
		imp_->cart_param->log()->setPlanPreError(int(ErrorCode2::eParamError),*this,"",e.what());
		option() |= aris::plan::Plan::Option::NOT_RUN_EXECUTE_FUNCTION;
		return;
	}
	
	for (auto &option : motorOptions()) option |= Plan::USE_TARGET_POS;

}
auto FollowerCart::executeRT()->int 
{

	// 规划算法
	imp_->ret = 0;

	if(!imp_->udpf->isRunning())
	{
        for(std::size_t i = 0;i < imp_->cart_param->ctrlEes().size();i++)
        {
            double p_next[6]{ 0,0,0,0,0,0 }, v_next[6]{ 0,0,0,0,0,0 }, a_next[6]{ 0,0,0,0,0,0 };
            aris::Size t;

            for (size_t j = 0; j < 6; j++)
            {
                imp_->ret += aris::plan::moveAbsolute2(imp_->now_pos[i][j], imp_->now_vel[i][j], imp_->now_acc[i][j]
                    , imp_->target_pos[i][j] , 0.0, 0.0
                    , imp_->vel[i][j], imp_->acc[i][j], imp_->acc[i][j]
                    , imp_->dt * 1e-3, 1e-10, p_next[j], v_next[j], a_next[j], t);
            }

            aris::dynamic::s_vc(6, p_next, imp_->cur_pos[i].data());
            aris::dynamic::s_vs(6,imp_->now_pos[i].data(),imp_->cur_pos[i].data());

            // 保存下个周期的copy //
            aris::dynamic::s_vc(6, p_next, imp_->now_pos[i].data());
            aris::dynamic::s_vc(6, v_next, imp_->now_vel[i].data());
            aris::dynamic::s_vc(6, a_next, imp_->now_acc[i].data());

        }

        if (!imp_->ret) return imp_->ret;
        
	}
	else //正常运行
	{
        imp_->cart_param->getDataFromQueue(imp_->target_pos);
        for(std::size_t i = 0;i < imp_->cart_param->ctrlEes().size();i++)
        {
            double p_next[6]{ 0,0,0,0,0,0 }, v_next[6]{ 0,0,0,0,0,0 }, a_next[6]{ 0,0,0,0,0,0 };
            aris::Size t;

            for (size_t j = 0; j < 6; j++)
            {

                double real_vel = 0;
                if (imp_->cart_param->robot()->robotOpMode() == kaanh::robot::RobotOpMode::kManual)
                    real_vel = imp_->cart_param->robot()->vel() / 100.0;
                else
                    real_vel = imp_->cart_param->robot()->pgmVel() / 100.0;

                imp_->ret += aris::plan::moveAbsolute2(imp_->now_pos[i][j], imp_->now_vel[i][j], imp_->now_acc[i][j]
                    , imp_->target_pos[i][j] , 0.0, 0.0
                    , real_vel * imp_->vel[i][j], imp_->acc[i][j], imp_->acc[i][j]
                    , imp_->dt * 1e-3, 1e-10, p_next[j], v_next[j], a_next[j], t);
            }
            aris::dynamic::s_vc(6, p_next, imp_->cur_pos[i].data());
            aris::dynamic::s_vs(6,imp_->now_pos[i].data(),imp_->cur_pos[i].data());

            // 保存下个周期的copy //
            aris::dynamic::s_vc(6, p_next, imp_->now_pos[i].data());
            aris::dynamic::s_vc(6, v_next, imp_->now_vel[i].data());
            aris::dynamic::s_vc(6, a_next, imp_->now_acc[i].data());

        }
    
        if (!imp_->ret)
        {           
            if(count()%100 == 0)
                mout()<< "waiting...." << std::endl;
            return 1;
        }
	}

    //-------------------------设置末端-----------------------//
    for (size_t i=0;i<imp_->cart_param->ctrlEes().size();i++)
    {
        auto& model = imp_->cart_param->multiModel()->subModels()[imp_->cart_param->twManager()->getModelId(imp_->cart_param->ctrlEeId()[i])];
        switch (imp_->cart_param->ctrlEeTypes()[i])
        {
            case aris::dynamic::EEType::PE313: [[fallthrough]];
            case aris::dynamic::EEType::PE321: [[fallthrough]];
            case aris::dynamic::EEType::PE123: [[fallthrough]];
            case aris::dynamic::EEType::PM: [[fallthrough]];
            case aris::dynamic::EEType::PQ:	[[fallthrough]];
            case aris::dynamic::EEType::RE313: [[fallthrough]];
            case aris::dynamic::EEType::RE321: [[fallthrough]];
            case aris::dynamic::EEType::RE123: [[fallthrough]];
            case aris::dynamic::EEType::RM: [[fallthrough]];
            case aris::dynamic::EEType::RQ:	[[fallthrough]];
            case aris::dynamic::EEType::XYZT:[[fallthrough]];
            case aris::dynamic::EEType::XYZ:{
                //  绝对坐标系 获取当前位姿矩阵 //
				std::memset(imp_->pm_now,0,sizeof(double)*16);
				std::memset(imp_->pm_temp,0,sizeof(double)*16);
				std::memset(imp_->pm,0,sizeof(double)*16);
				std::memset(imp_->pm_target,0,sizeof(double)*16);

                aris::dynamic::s_pe2pm(imp_->cur_pos[i].data(), imp_->pm,"321");
                if(imp_->cart_param->twManager()->toolMode()[imp_->cart_param->ctrlEeId()[i]] == ToolMode::kTool)
                {
                    if (imp_->cor == kaanh::robot::JogCoordinate::kBase) {
                        imp_->tool[i]->getPm(*imp_->wobj0[i], imp_->pm_now);
                        aris::dynamic::s_mm(3, 3, 3, imp_->pm, 4, imp_->pm_now, 4, imp_->pm_target, 4);
                        aris::dynamic::s_vc(3, imp_->pm + 3, 4, imp_->pm_target + 3, 4);
                        aris::dynamic::s_va(3, imp_->pm_now + 3, 4, imp_->pm_target + 3, 4);
                        imp_->tool[i]->setPm(*imp_->wobj0[i], imp_->pm_target);
                    }
                    else if (imp_->cor == kaanh::robot::JogCoordinate::kWobj) {
                        imp_->tool[i]->getPm(*imp_->wobj[i], imp_->pm_now);
                        aris::dynamic::s_mm(3, 3, 3, imp_->pm, 4, imp_->pm_now, 4, imp_->pm_target, 4);
                        aris::dynamic::s_vc(3, imp_->pm + 3, 4, imp_->pm_target + 3, 4);
                        aris::dynamic::s_va(3, imp_->pm_now + 3, 4, imp_->pm_target + 3, 4);
                        imp_->tool[i]->setPm(*imp_->wobj[i], imp_->pm_target);
                    }
                    else if (imp_->cor == kaanh::robot::JogCoordinate::kTool) {
                        imp_->tool[i]->getPm(*imp_->wobj[i], imp_->pm_now);
                        aris::dynamic::s_pm_dot_pm(imp_->pm_now, imp_->pm, imp_->pm_target);
                        imp_->tool[i]->setPm(*imp_->wobj[i], imp_->pm_target);

                    }
                }
                else
                {
                    if (imp_->cor == kaanh::robot::JogCoordinate::kBase) {
                        imp_->ex_tool[i]->getPm(*imp_->tool0[i], imp_->pm_now);
                        aris::dynamic::s_mm(3, 3, 3, imp_->pm, 4, imp_->pm_now, 4, imp_->pm_temp, 4);
                        aris::dynamic::s_vc(3, imp_->pm + 3, 4, imp_->pm_temp + 3, 4);
                        aris::dynamic::s_va(3, imp_->pm_now + 3, 4, imp_->pm_temp + 3, 4);

                        aris::dynamic::s_inv_pm(imp_->pm_temp, imp_->pm_target);
                        imp_->tool0[i]->setPm(*imp_->ex_tool[i], imp_->pm_target);
                    }
                    else if (imp_->cor == kaanh::robot::JogCoordinate::kWobj) {
                        imp_->ex_tool[i]->getPm(*imp_->hd_wobj[i], imp_->pm_now);
                        aris::dynamic::s_mm(3, 3, 3, imp_->pm, 4, imp_->pm_now, 4, imp_->pm_temp, 4);
                        aris::dynamic::s_vc(3, imp_->pm + 3, 4, imp_->pm_temp + 3, 4);
                        aris::dynamic::s_va(3, imp_->pm_now + 3, 4, imp_->pm_temp + 3, 4);

                        aris::dynamic::s_inv_pm(imp_->pm_temp, imp_->pm_target);
                        imp_->hd_wobj[i]->setPm(*imp_->ex_tool[i], imp_->pm_target);
                    }
                    else if (imp_->cor == kaanh::robot::JogCoordinate::kTool) {
                        imp_->ex_tool[i]->getPm(*imp_->hd_wobj[i], imp_->pm_now);
                        aris::dynamic::s_pm_dot_pm(imp_->pm_now, imp_->pm, imp_->pm_temp);
                        aris::dynamic::s_inv_pm(imp_->pm_temp, imp_->pm_target);
                        imp_->hd_wobj[i]->setPm(*imp_->ex_tool[i], imp_->pm_target);
                    }
                }

                imp_->cart_param->ctrlEes()[i]->updP();

                //运动学反解//
                if (model.inverseKinematics()) {
					imp_->cart_param->log()->setPlanExeError(int(ErrorCode2::eInverseKinematics),*this,"FollowerCart");
                }
                break;
            }
            case aris::dynamic::EEType::A:[[fallthrough]];
            case aris::dynamic::EEType::X:{
				double cur_pos;
				imp_->cart_param->ctrlEes().at(i)->getP(&cur_pos);
				aris::dynamic::s_va(1,imp_->cur_pos[i].data(),&cur_pos);
                imp_->cart_param->ctrlEes().at(i)->setP(&cur_pos);
                //运动学反解//
                if (model.inverseKinematics()) {
					imp_->cart_param->log()->setPlanExeError(int(ErrorCode2::eInverseKinematics),*this,"FollowerCart");
                }

                break;
            }
            default:
                break;
        }
    
    
        if ( model.isSingular(0.05))
        {
            imp_->cart_param->log()->setPlanExeError(int(ErrorCode2::eCloseToSingularPoints),*this,TR_STR("Robot '{1}'",std::to_string(imp_->cart_param->twManager()->getModelId(imp_->cart_param->ctrlEeId()[i]))));
        }
    }	

	return imp_->ret;
}
auto FollowerCart::collectNrt()->void {
    imp_->udpf->setRunningState(false);
}
FollowerCart::~FollowerCart() = default;
FollowerCart::FollowerCart(const std::string &name) :imp_(new Imp) {
	aris::core::fromXmlString(command(),
				"<Command name=\"follower_cart\">"
			"	<GroupParam>"
			"		<UniqueParam>"
			"			<GroupParam>"
			"				<Param name=\"increase_count\" default=\"500\"/>"
			"				<Param name=\"dimdir\" default=\"{0}\"/>"
			"				<Param name=\"ee_ids\" default=\"{0}\"/>"
			"			</GroupParam>"
			"			<Param name=\"stop\"/>"
			"		</UniqueParam>"
			"	</GroupParam>"
			"</Command>");

}
KAANH_DEFINE_BIG_FOUR_CPP(FollowerCart)

ARIS_REGISTRATION
{

    typedef aris::core::SocketServer& (FollowerParamBase::*SocketServerFunc)();
    aris::core::class_<FollowerParamBase>("FollowerParamBase")
        .prop("socket_server", &FollowerParamBase::setSocketServer, (SocketServerFunc)(&FollowerParamBase::socketServer))
        .prop("tg_id", &FollowerParamBase::setTgId , &FollowerParamBase::tgId);
    aris::core::class_<FollowerAjParam>("FollowerAjParam")
        .inherit<FollowerParamBase>();
    aris::core::class_<FollowerCartParam>("FollowerCartParam")
        .inherit<FollowerParamBase>();



    aris::core::class_<aris::core::PointerArray<FollowerParamBase>>("FollowerParamPool")
        .asRefArray();


    typedef aris::core::PointerArray<FollowerParamBase>& (UDPFollower::*FollowerParamPoolFunc)();
    aris::core::class_<UDPFollower>("UDPFollower")
    .inherit<kaanh::module::MiddleModule>()
    .prop("follower_param_pool", &UDPFollower::resetFollowerParamPool, (FollowerParamPoolFunc)(&UDPFollower::followerParamPool))
    ;

    aris::core::class_<FollowerAjTg>("kaanh::FollowerAjTg").inherit<aris::plan::Plan>();
    aris::core::class_<FollowerAj>("kaanh::FollowerAj").inherit<aris::plan::Plan>();
    aris::core::class_<FollowerAjAbsolute2>("kaanh::FollowerAjAbsolute2").inherit<aris::plan::Plan>();
    aris::core::class_<FollowerCart>("kaanh::FollowerCart").inherit<aris::plan::Plan>();



}

}



