use r2r;
use failure::Error;

// try to run like this
// ros2 run r2r_minimal_node r2r_minimal_node --ros-args -p param1:=[str1,str2] -p param2:=5.5 param3=true -r __ns:=/demo -r __node:=my_node

fn main() -> Result<(), Error> {
    let ctx = r2r::Context::create()?;
    let node = r2r::Node::create(ctx, "testnode", "")?;

    println!("node name: {}", node.name()?);
    println!("node fully qualified name: {}", node.fully_qualified_name()?);

    let msg = r2r::std_msgs::msg::String { data: "hello".into() };
    println!("msg: {:?}", msg);

    println!("node parameters");
    node.params.iter().for_each(|(k,v)| {
        println!("{} - {:?}", k, v);
    });

    Ok(())
}
