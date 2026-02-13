import asyncio
import aiohttp
import os
from paranet_agent import actor, connector
from paranet_agent.actor import BaseActor, Conversation

jira_base_url = os.getenv("JIRA_BASE_URL", "JIRA_BASE_URL")
jira_api_email = os.getenv("JIRA_API_EMAIL", "JIRA_API_EMAIL")
jira_api_token = os.getenv("JIRA_API_TOKEN", "JIRA_API_TOKEN")
jira_project_key = os.getenv("JIRA_PROJECT_KEY", "JIRA_PROJECT_KEY")

@actor.type
class TaskStatus:
    status: str
    ticket: str = None

@actor.actor
class Jira(BaseActor):
    @actor.skill(subject='jira', response=TaskStatus)
    def create_ticket(self, conv: Conversation) -> None:
        asyncio.ensure_future(self.create_jira_ticket(conv))

    @actor.skill(subject='jira', response=TaskStatus)
    def comment_ticket(self, ticket: str, comment: str, conv: Conversation) -> None:
        asyncio.ensure_future(self.add_comment_to_ticket(ticket, comment, conv))

    @actor.skill(subject='jira', response=TaskStatus)
    def update_ticket_status(self, ticket: str, status: str, conv: Conversation) -> None:
        asyncio.ensure_future(self.transition_ticket_status(ticket, status, conv))
    
    @actor.skill(subject='jira', response=TaskStatus)
    def get_oldest_todo_ticket(self, conv: Conversation) -> None:
        asyncio.ensure_future(self.fetch_oldest_todo_ticket(conv))

    async def create_jira_ticket(self, conv):
        url = f"{jira_base_url}/rest/api/3/issue"
        headers = {"Accept": "application/json", "Content-Type": "application/json"}
        auth = aiohttp.BasicAuth(jira_api_email, jira_api_token)

        payload = {
            "fields": {
                "project": {"key": jira_project_key},
                "summary": f"New Work Order",
                "description": {
                    "version": 1,
                    "type": "doc",
                    "content": [
                        {"type": "paragraph", "content": [{"type": "text", "text": "Work order has been placed."}]}
                    ]
                },
                "issuetype": {"name": "Task"}
            }
        }

        async with aiohttp.ClientSession() as session:
            async with session.post(url, json=payload, headers=headers, auth=auth) as response:
                if response.status == 201:
                    data = await response.json()
                    ticket_id = data.get("key", "Unknown")
                    print(f"Jira ticket created: {ticket_id}")
                    conv.send_response(TaskStatus(status="done", ticket=ticket_id))
                else:
                    error_text = await response.text()
                    print(f"Failed to create Jira ticket: {error_text}")
                    conv.send_response(TaskStatus(status="failed", ticket=""))

    async def transition_ticket_status(self, ticket_id, new_status, conv):
        transition_map = {
            "inprogress": "21",
            "done": "31",
        }

        transition_id = transition_map.get(new_status.lower())
        if not transition_id:
            conv.send_response(TaskStatus(status="failed", ticket=ticket_id))
            return

        url = f"{jira_base_url}/rest/api/3/issue/{ticket_id}/transitions"
        headers = {"Accept": "application/json", "Content-Type": "application/json"}
        auth = aiohttp.BasicAuth(jira_api_email, jira_api_token)

        transition_payload = {"transition": {"id": transition_id}}

        async with aiohttp.ClientSession() as session:
            async with session.post(url, json=transition_payload, headers=headers, auth=auth) as response:
                if response.status == 204:
                    print(f"Transitioned ticket {ticket_id} to {new_status}")
                    conv.send_response(TaskStatus(status="done", ticket=ticket_id))
                else:
                    error_text = await response.text()
                    print(f"Failed to transition ticket {ticket_id}: {error_text}")
                    conv.send_response(TaskStatus(status="failed", ticket=ticket_id))

    async def add_comment_to_ticket(self, ticket_id, comment, conv):
        url = f"{jira_base_url}/rest/api/3/issue/{ticket_id}/comment"
        headers = {"Accept": "application/json", "Content-Type": "application/json"}
        auth = aiohttp.BasicAuth(jira_api_email, jira_api_token)

        payload = {
            "body": {
                "version": 1,
                "type": "doc",
                "content": [
                    {"type": "paragraph", "content": [{"type": "text", "text": comment}]}
                ]
            }
        }

        async with aiohttp.ClientSession() as session:
            async with session.post(url, json=payload, headers=headers, auth=auth) as response:
                if response.status == 201:
                    print(f"Comment added to ticket {ticket_id}.")
                    conv.send_response(TaskStatus(status="done", ticket=ticket_id))
                else:
                    error_text = await response.text()
                    print(f"Failed to add comment to ticket {ticket_id}: {error_text}")
                    conv.send_response(TaskStatus(status="failed", ticket=ticket_id))

    async def fetch_oldest_todo_ticket(self, conv):
        url = f"{jira_base_url}/rest/api/3/search"
        headers = {"Accept": "application/json"}
        auth = aiohttp.BasicAuth(jira_api_email, jira_api_token)

        jql = f'project = "{jira_project_key}" AND status = "Order Queue" ORDER BY created ASC'


        params = {
            "jql": jql,
            "fields": "key",
            "maxResults": 1
        }

        async with aiohttp.ClientSession() as session:
            async with session.get(url, headers=headers, auth=auth, params=params) as response:
                if response.status == 200:
                    data = await response.json()
                    issues = data.get("issues", [])
                    if issues:
                        ticket_id = issues[0]["key"]
                        print(f"Oldest 'Order Queue' ticket: {ticket_id}")
                        conv.send_response(TaskStatus(status="done", ticket=ticket_id))
                    else:
                        print(f"No 'Order Queue' tickets found.")
                        conv.send_response(TaskStatus(status="empty", ticket=""))
                else:
                    error_text = await response.text()
                    print(f"Failed to fetch 'Order Queue' tickets: {error_text}")
                    conv.send_response(TaskStatus(status="failed", ticket=""))

actor.register_actor(Jira())
connector.start()
actor.deploy('jira', restart=False)

loop = asyncio.get_event_loop()
loop.run_until_complete(connector.get_task())
