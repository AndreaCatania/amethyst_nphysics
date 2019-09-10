#[macro_export]
macro_rules! matches {
    ($e:expr, $p:pat) => {
        match $e {
            $p => true,
            _ => false,
        }
    };
}

/// These macros are used to assert the validity of some data, but instead to panics they stop
/// the functions processing print a message on the console.
///
/// You can use the macro fail_cond(expression, return) if you need to return a value in case of fail.
#[macro_export]
macro_rules! fail_cond {
    ($x:expr) => {{
        if $x {
            use log::error;
            error!(
                "[{}::{}] The expression `{}` failed execution.",
                file!(),
                line!(),
                stringify!($x)
            );
            return;
        }
    }};
    ($x:expr, $y:expr) => {{
        if $x {
            use log::error;
            error!(
                "[{}::{}] The expression `{}` failed execution. Return {}.",
                file!(),
                line!(),
                stringify!($x),
                stringify!($y)
            );
            return $y;
        }
    }};
}

#[macro_export]
macro_rules! fail {
    ($x:tt) => {
        use log::error;
        error!("[{}::{}] {}", file!(), line!(), stringify!($x),);
        return;
    };
    ($x:tt, $ret:expr) => {
        use log::error;
        error!("[{}::{}] {}", file!(), line!(), stringify!($x),);
        return $ret;
    };
}
