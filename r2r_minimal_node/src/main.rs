use r2r;
use r2r::r2r_minimal_node_msgs::srv::HelloWorld;

fn handle_service(request: HelloWorld::Request) -> HelloWorld::Response {
    println!("request: {}", request.hello);
    HelloWorld::Response {
        world: if request.hello == "Hello" {
            "World!".into()
        } else {
            "Hello Rust".into()
        }
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "testnode", "")?;

    println!("node name: {}", node.name()?);
    println!("node fully qualified name: {}", node.fully_qualified_name()?);

    let msg = r2r::std_msgs::msg::String { data: "hello".into() };
    println!("msg: {:?}", msg);

    println!("node parameters");
    node.params.iter().for_each(|(k,v)| {
        println!("{} - {:?}", k, v);
    });

    // Run our custom service
    node.create_service::<HelloWorld::Service>("/hello_world", Box::new(handle_service))?;

    // create custom msg and action messages just to test that they are built
    let hello_msg = r2r::r2r_minimal_node_msgs::msg::Hello::default();
    assert_eq!(hello_msg.hello, String::new());

    let hello_goal_msg = r2r::r2r_minimal_node_msgs::action::SayHello::Goal::default();
    assert_eq!(hello_goal_msg.to_say, String::new());

    loop {
        node.spin_once(std::time::Duration::from_millis(100));
    }
}
