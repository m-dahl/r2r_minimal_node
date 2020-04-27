use r2r;
use failure::Error;

// try to run like this
// ros2 run r2r_minimal_node r2r_minimal_node --ros-args -p param_key:=[hej,hopp] -p key2:=5.5 key2=true -r __ns:=/demo -r __node:=my_node

fn main() -> Result<(), Error> {
    let ctx = r2r::Context::create()?;
    let node = r2r::Node::create(ctx, "testnode", "")?;

    println!("node name: {}", node.name()?);
    println!("node fully qualified name: {}", node.fully_qualified_name()?);

    println!("node parameters");
    node.params.iter().for_each(|(k,v)| {
        println!("{} - {:?}", k, v);
    });

    Ok(())
}
