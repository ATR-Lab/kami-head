"""Announcement templates for virtual requests"""

# Templates for formatting virtual request announcements
REQUEST_ANNOUNCEMENT_TEMPLATES = {
    "NEW_COFFEE_REQUEST": "excited:New order alert! We have a {content} request coming in!",
    "ORDER_READY": "professional:Order ready for pickup: {content}!",
    "ORDER_PROCESSING": "helpful:Order update: {content} is now being prepared!",
    "ORDER_COMPLETED": "cheerful:Great news! {content} has been completed and delivered!",
    "ORDER_UPDATED": "friendly:Order update: {content}",
    "CUSTOMER_WAITING": "helpful:Customer notification: {content}",
    "DEFAULT": "friendly:Update: {content}"
}


def format_virtual_request_announcement(request: dict) -> str:
    """Format virtual request as emotional announcement"""
    request_type = request["type"]
    content = request["content"]
    
    # Get template from dictionary, default to "DEFAULT" if type not found
    template = REQUEST_ANNOUNCEMENT_TEMPLATES.get(request_type, REQUEST_ANNOUNCEMENT_TEMPLATES["DEFAULT"])
    return template.format(content=content) 