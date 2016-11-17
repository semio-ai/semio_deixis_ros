#include <ros/ros.h>

#include <semio_msgs_ros/DeicticRecognitionResult.h>
#include <semio_msgs_ros/DeicticTargets.h>

#include <semio/recognition/deictic_recognizer.h>
#include <semio/ros/humanoid_source_adapter.h>

//! Simple ROS wrapper around semio::DeicticRecognizer
class SemioDeixisNode
{
public:
    //! ROS message for the result of deictic recognition for all humanoids
    typedef semio_msgs_ros::DeicticRecognitionResult _DeicticRecognitionResultMsg;
    //! ROS message for the result of deictic recognition for a single humanoid
    typedef semio_msgs_ros::DeicticRecognitionHumanoidItem _DeicticRecognitionHumanoidItemMsg;
    //! ROS message for the result of deictic recognition for a single humanoid joint
    typedef semio_msgs_ros::DeicticRecognitionSourceItem _DeicticRecognitionSourceItemMsg;
    //! ROS message for the result of deictic recognition for a single deictic target
    typedef semio_msgs_ros::DeicticRecognitionTopNItem _DeicticRecognitionTopNItemMsg;
    //! ROS message for a vector of deictic targets
    typedef semio_msgs_ros::DeicticTargets _DeicticTargetsMsg;
    //! ROS message for a single deictic target
    typedef semio_msgs_ros::DeicticTarget _DeicticTargetMsg;

    //! NodeHandle copy used to interface with ROS
    ros::NodeHandle nh_rel_;
    //! Deictic recognition result publisher
    ros::Publisher result_pub_;
    //! Deictic targets subscriber
    ros::Subscriber targets_sub_;

    //! Pointer to the input source for humanoids
    semio::HumanoidSource::Ptr humanoid_source_ptr_;
    //! Semio deictic recognizer
    semio::DeicticRecognizer deictic_recognizer_;

    /**
    @param nh_rel @copybrief nh_rel_
    @param humanoid_source_ptr @copybrief humanoid_source_ptr_
    */
    SemioDeixisNode( ros::NodeHandle & nh_rel, semio::HumanoidSource::Ptr humanoid_source_ptr )
    :
        nh_rel_( nh_rel ),
        result_pub_( nh_rel_.advertise<_DeicticRecognitionResultMsg>( "result", 100 ) ),
        targets_sub_( nh_rel_.subscribe( "targets", 10, &SemioDeixisNode::targetsCB, this ) ),
        humanoid_source_ptr_( humanoid_source_ptr )
    {
        typedef std::function<void(std::string const &)> _ParamFunc;
        typedef std::pair<std::string, _ParamFunc> _ParamOp;
        for( auto const & param_op : {
            _ParamOp(
                "filter/max_size",
                [this]( std::string const & param_name ){
                    this->deictic_recognizer_.setFilterMaxSize( this->nh_rel_.param<int>( std::string( param_name ), 0 ) );
                } ),
            _ParamOp( "filter/max_duration",
                [this]( std::string const & param_name ){
                    this->deictic_recognizer_.setFilterMaxDuration( this->nh_rel_.param<double>( std::string( param_name ), 0 ) );
                } ) } )
        {
            if( nh_rel_.hasParam( param_op.first ) ) param_op.second( param_op.first );
        }
    }

    //! Main loop
    void spin()
    {
        ros::Rate loop_rate( 30 );

        while( ros::ok() )
        {
            //! - Trigger ROS callbacks
            ros::spinOnce();

            //! - Pass humanoids to deictic recognizer
            deictic_recognizer_.getHumanoids() = humanoid_source_ptr_->update();
            //! - Calculate deictic recognition result
            semio::DeicticRecognitionResult const & result( deictic_recognizer_.calculateResult() );

            //----------
            //! - Convert deictic recognition result to ROS message
            _DeicticRecognitionResultMsg result_msg;

            result_msg.humanoids.reserve( result.size() );

            for( auto const & humanoid_item : result )
            {
                semio::DeixisSourceMap const & sources( humanoid_item.second );

                _DeicticRecognitionHumanoidItemMsg humanoid_msg;
                humanoid_msg.id = static_cast<uint32_t>( humanoid_item.first );
                humanoid_msg.sources.reserve( sources.size() );

                for( auto const & source_item : sources )
                {
                    semio::DeixisTopNList const & top_n_list( source_item.second );

                    _DeicticRecognitionSourceItemMsg source_msg;
                    source_msg.name = source_item.first;
                    source_msg.top_n_list.reserve( top_n_list.size() );

                    for( auto const & list_item : top_n_list.values() )
                    {
                        _DeicticRecognitionTopNItemMsg top_n_item_msg;
                        top_n_item_msg.likelihood = list_item.getValue();
                        top_n_item_msg.target_name = list_item.getData();

                        source_msg.top_n_list.push_back( std::move( top_n_item_msg ) );
                    }

                    humanoid_msg.sources.push_back( std::move( source_msg ) );
                }

                result_msg.humanoids.push_back( std::move( humanoid_msg ) );
            }
            //----------

            //! - Publish deictic recognition result
            result_pub_.publish( std::move( result_msg ) );

            loop_rate.sleep();
        }
    }

    //! ROS callback for deictic targets
    /**
    @param msg_ptr ConstPtr to the deictic target message
    */
    void targetsCB( _DeicticTargetsMsg::ConstPtr const & msg_ptr )
    {
        auto const & targets_msg = *msg_ptr;

        //----------
        //! - Convert deictic target message to semio::DeicticTargetArray
        semio::DeicticTargetArray deictic_targets;

        for( auto const & target_msg : targets_msg.targets )
        {
            auto const & name = target_msg.name;
            auto const & position = target_msg.position;

            deictic_targets.emplace( name, Eigen::Vector3d( position.x, position.y, position.z ) );
        }
        //----------

        //! - Update deictic recognizer's list of targets
        deictic_recognizer_.getTargets() = deictic_targets;
    }
};

int main( int argc, char ** argv )
{
    ros::init( argc, argv, "semio_deixis_node" );
    //! - Create NodeHandle with relative namespace
    ros::NodeHandle nh_rel( "~" );

    //! - Create semio::ros::HumanoidSourceAdapter
    semio::ros::HumanoidSourceAdapter humanoid_source_adapter( nh_rel );

    //! - Create SemioDeixisNode; pass node handle and humanoid source
    SemioDeixisNode semio_deixis_node( nh_rel, humanoid_source_adapter.getHumanoidSource() );
    //! - Start main loop SemioDeixisNode::spin()
    semio_deixis_node.spin();

    return 0;
}
