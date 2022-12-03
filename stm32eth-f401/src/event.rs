use core::task::Poll;
use core::future::Future;

pub type BtnPressConsumer = heapless::spsc::Consumer<'static, (), 4>;

pub enum ExternalEvent {
    ButtonPressed
}

pub struct ButtonPressFuture<'a>(&'a mut BtnPressConsumer);

pub /* async */ fn wait_btn_press<'a>(consumer: &'a mut BtnPressConsumer) -> ButtonPressFuture<'a> {
    ButtonPressFuture(consumer)
}

impl<'a> Future for ButtonPressFuture<'a> {
    type Output = ExternalEvent;
    fn poll(mut self: core::pin::Pin<&mut Self>, ctx: &mut core::task::Context<'_>) -> Poll<ExternalEvent> {
        match self.0.dequeue() {
            Some(_) => Poll::Ready(ExternalEvent::ButtonPressed),
            None => {
                ctx.waker().wake_by_ref();
                Poll::Pending
            }
        }
    }
}
